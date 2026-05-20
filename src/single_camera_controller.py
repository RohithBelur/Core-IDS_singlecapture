# single_camera_controller.py

import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple, List, Dict

import numpy as np
from imageio.v2 import imwrite

from ids_peak import ids_peak
from ids_peak_ipl import ids_peak_ipl as ipl
from ids_peak import ids_peak_ipl_extension as ipl_ext
from ids_peak_afl import ids_peak_afl as afl


# -------------------------
# Image metrics (no OpenCV)
# -------------------------

def sharpness_laplacian_var(gray: np.ndarray) -> float:
    gray = gray.astype(np.float32)
    if gray.shape[0] < 3 or gray.shape[1] < 3:
        return 0.0
    lap = (
        -4.0 * gray[1:-1, 1:-1]
        + gray[1:-1, 0:-2]
        + gray[1:-1, 2:]
        + gray[0:-2, 1:-1]
        + gray[2:, 1:-1]
    )
    return float(lap.var())


def sharpness_mean_grid(gray: np.ndarray, frac: float = 0.25) -> float:
    h, w = gray.shape[:2]
    rh = max(40, int(h * frac))
    rw = max(40, int(w * frac))

    ys = [h // 6, h // 2, (5 * h) // 6]
    xs = [w // 6, w // 2, (5 * w) // 6]

    vals = []
    for cy in ys:
        for cx in xs:
            y0 = max(0, min(h - rh, cy - rh // 2))
            x0 = max(0, min(w - rw, cx - rw // 2))
            roi = gray[y0:y0 + rh, x0:x0 + rw]
            vals.append(sharpness_laplacian_var(roi))
    return float(np.mean(vals)) if vals else 0.0


def clip_fractions(gray: np.ndarray, lo: int = 0, hi: int = 255) -> Tuple[float, float]:
    return float(np.mean(gray <= lo)), float(np.mean(gray >= hi))


def _node_value(node):
    if node is None:
        return None
    for fn in ("Value", "GetValue"):
        try:
            return getattr(node, fn)()
        except Exception:
            pass
    return None


def _try_set_enum(nodemap, name: str, entry: str) -> bool:
    try:
        node = nodemap.FindNode(name)
        if node is None:
            return False
        node.SetCurrentEntry(entry)
        return True
    except Exception:
        return False


def _try_execute(nodemap, name: str) -> bool:
    try:
        node = nodemap.FindNode(name)
        if node is None:
            return False
        node.Execute()
        return True
    except Exception:
        return False


def _find_first_node(nodemap, names: List[str]):
    for n in names:
        try:
            node = nodemap.FindNode(n)
            if node is not None:
                return n, node
        except Exception:
            pass
    return None, None


def _safe_call(obj, name: str):
    fn = getattr(obj, name, None)
    if fn is None:
        return None
    try:
        return fn()
    except Exception:
        return None


@dataclass
class FrameStats:
    mean: float
    lo_clip: float
    hi_clip: float
    sharp: float
    mn: int
    mx: int


class SingleCameraController:
    """
    Single IDS camera controller with:
      - robust free-run acquisition
      - AFL autofocus (if available)
      - MANUAL exposure/gain safety (prevents "all black" sequences)
      - best-frame selection by sharpness

    Key design points:
      - TriggerMode Off
      - If AFL Brightness "ONCE" is not supported, we do NOT use it
      - We force ExposureTime/Gain to sane values each capture
      - We soft-restart the stream if we detect repeated invalid frames
    """

    def __init__(
        self,
        serial: Optional[str] = None,
        # capture_dir: str = "/data/captures",
        capture_dir: str = "/home/spacetime/Documents/Core-IDS_SingleCapture/spot-coreio-single-camera/CaptureTemp",
        # Manual safety defaults (you can override via env)
        exposure_us: int = 20000,           # <-- IMPORTANT: pick a sane manual default for your scene
        exposure_min_us: int = 100,
        exposure_max_us: int = 20000,
        gain: float = 8.0,
        gain_min: float = 0.0,
        gain_max: float = 10.0,
        focus_min: int = 400,
        focus_max: int = 1000,
    ):
        self.requested_serial = serial or os.environ.get("CAMERA_SERIAL")
        self.capture_dir = Path(os.environ.get("CAPTURE_DIR", capture_dir))
        self.capture_dir.mkdir(parents=True, exist_ok=True)

        # Env overrides
        self.exposure_us = int(os.environ.get("EXPOSURE_US", exposure_us))
        self.exposure_min_us = int(os.environ.get("EXPOSURE_MIN_US", exposure_min_us))
        self.exposure_max_us = int(os.environ.get("EXPOSURE_MAX_US", exposure_max_us))

        self.gain = float(os.environ.get("GAIN", gain))
        self.gain_min = float(os.environ.get("GAIN_MIN", gain_min))
        self.gain_max = float(os.environ.get("GAIN_MAX", gain_max))

        self.focus_min = int(os.environ.get("FOCUS_MIN", focus_min))
        self.focus_max = int(os.environ.get("FOCUS_MAX", focus_max))

        self.device = None
        self.nodemap = None
        self.ds = None
        self._buffers = []

        # AFL
        self.afl_mgr = None
        self._af = None                 # autofocus controller
        self._bc = None                 # brightness controller (optional)
        self._afl_enabled = False
        self._use_brightness = (os.environ.get("USE_AFL_BRIGHTNESS", "0") == "1")

        # Nodes (logging + manual set)
        self._focus_node = None
        self._exposure_node = None
        self._gain_node = None
        self._exposure_auto_node = None
        self._gain_auto_node = None

        self._invalid_streak = 0

    # -------------------------
    # Init / Stream setup
    # -------------------------

    def _trigger_software(self) -> None:
        node = self.nodemap.FindNode("TriggerSoftware")
        if node is None:
            raise RuntimeError("TriggerSoftware node not found (cannot software-trigger).")
        node.Execute()

    def initialize(self):
        ids_peak.Library.Initialize()
        dm = ids_peak.DeviceManager.Instance()
        dm.Update()
        devs = dm.Devices()
        if devs.empty():
            raise RuntimeError("No IDS camera detected (DeviceManager.Devices() empty).")

        wanted = str(self.requested_serial) if self.requested_serial else None
        matching_desc = None
        detected = []

        for desc in devs:
            try:
                dev = desc.OpenDevice(ids_peak.DeviceAccessType_Control)
                nm = dev.RemoteDevice().NodeMaps()[0]
                sn = _node_value(nm.FindNode("DeviceSerialNumber"))
                sn = str(sn) if sn is not None else ""
                detected.append(sn or "<unreadable>")
                dev = None

                if wanted is None:
                    print(f"[Camera] Using first available device (serial={sn or '<unreadable>'})")
                    matching_desc = desc
                    break
                if sn == wanted:
                    print(f"[Camera] Using device with serial={sn}")
                    matching_desc = desc
                    break
            except Exception as e:
                print(f"[Camera] Failed to probe device: {e}")
                continue

        if matching_desc is None:
            raise RuntimeError(f"No IDS device matching serial={wanted} found. Detected: {detected}")

        # Open
        try:
            self.device = matching_desc.OpenDevice(ids_peak.DeviceAccessType_Exclusive)
            print("[Camera] Opened in Exclusive mode.")
        except Exception as e:
            print(f"[Camera] Exclusive open failed ({e}); trying Control mode…")
            self.device = matching_desc.OpenDevice(ids_peak.DeviceAccessType_Control)
            print("[Camera] Opened in Control mode.")

        self.nodemap = self.device.RemoteDevice().NodeMaps()[0]

        # Try lock TL params
        try:
            self.nodemap.FindNode("TLParamsLocked").SetValue(1)
        except Exception:
            pass

        # Find useful nodes
        _, self._focus_node = _find_first_node(
            self.nodemap,
            ["FocusStepperValueControl", "Focus", "FocusPosition", "LensPosition"]
        )
        exp_name, self._exposure_node = _find_first_node(self.nodemap, ["ExposureTime", "ExposureTimeAbs"])
        gain_name, self._gain_node = _find_first_node(self.nodemap, ["Gain", "GainRaw", "GainAbs"])
        _, self._exposure_auto_node = _find_first_node(self.nodemap, ["ExposureAuto"])
        _, self._gain_auto_node = _find_first_node(self.nodemap, ["GainAuto"])

        print(f"[Camera] Focus node: {None if self._focus_node is None else 'present'}")
        print(f"[Camera] Exposure node: {exp_name or None}")
        print(f"[Camera] Gain node: {gain_name or None}")

        # AcquisitionMode=Continuous if possible (not fatal if missing)
        if not _try_set_enum(self.nodemap, "AcquisitionMode", "Continuous"):
            print("[Camera] Failed set AcquisitionMode=Continuous")

        # --- Trigger: Software (single-frame style like your working script) ---
        # Some cameras use TriggerSelector/TriggerSource; try best-effort.
        _try_set_enum(self.nodemap, "TriggerSelector", "FrameStart")
        _try_set_enum(self.nodemap, "TriggerSource", "Software")
        if _try_set_enum(self.nodemap, "TriggerMode", "On"):
            print("[Camera] Set TriggerMode=On (software trigger)")
        else:
            print("[Camera] Failed to set TriggerMode=On; will still try TriggerSoftware")

        trig_val = None
        try:
            trig_val = _node_value(self.nodemap.FindNode("TriggerMode"))
        except Exception:
            pass
        print(f"[Camera] TriggerMode: {trig_val}")
        print("[Camera] Trigger: Software.")

        # Force manual exp/gain baseline (prevents the “13.85” micro-exposure trap)
        self._force_manual_exposure_gain(tag="INIT")

        # DataStream
        self.ds = self.device.DataStreams()[0].OpenDataStream()

        payload_node = self.nodemap.FindNode("PayloadSize")
        payload = int(_node_value(payload_node))

        num_buffers = int(os.environ.get("IDS_NUM_BUFFERS", "10"))
        self._buffers = []
        for _ in range(num_buffers):
            b = self.ds.AllocAndAnnounceBuffer(payload)
            self._buffers.append(b)
            self.ds.QueueBuffer(b)

        self.ds.StartAcquisition()
        if not _try_execute(self.nodemap, "AcquisitionStart"):
            raise RuntimeError("Failed to execute AcquisitionStart")

        # AFL init + configure (AF only by default)
        self._configure_afl()

        print("[Camera] Initialization complete.")

    # -------------------------
    # Manual exposure/gain safety
    # -------------------------

    def _set_float_node(self, node, value: float) -> bool:
        if node is None:
            return False
        for fn in ("SetValue", "Set"):
            try:
                getattr(node, fn)(value)
                return True
            except Exception:
                pass
        return False

    def _set_enum_node(self, node, entry: str) -> bool:
        if node is None:
            return False
        try:
            node.SetCurrentEntry(entry)
            return True
        except Exception:
            return False

    def _force_manual_exposure_gain(self, tag: str):
        # Turn off auto if present
        if self._exposure_auto_node is not None:
            self._set_enum_node(self._exposure_auto_node, "Off")
        if self._gain_auto_node is not None:
            self._set_enum_node(self._gain_auto_node, "Off")

        # ExposureTime units vary by camera (often us; sometimes seconds).
        # Strategy:
        #   - Try set in us first
        #   - Read back; if it’s tiny (< 1.0) then it’s probably seconds, retry in seconds.
        if self._exposure_node is not None:
            target = float(max(self.exposure_min_us, min(self.exposure_us, self.exposure_max_us)))
            self._set_float_node(self._exposure_node, target)

        # Gain
        if self._gain_node is not None:
            g = float(max(self.gain_min, min(self.gain, self.gain_max)))
            self._set_float_node(self._gain_node, g)

        st = self._read_state()
        print(f"[Camera] [{tag}] Manual exp/gain enforced: exp={st['exposure']} gain={st['gain']}")

    # -------------------------
    # AFL configuration (AF only by default)
    # -------------------------

    def _configure_afl(self):
        # Initialize AFL library
        try:
            afl.Library.Initialize()
        except Exception:
            try:
                afl.Library.Init()
            except Exception:
                pass

        try:
            self.afl_mgr = afl.Manager(self.device.RemoteDevice().NodeMaps()[0])

            # Always try AF
            self._af = self.afl_mgr.CreateController(afl.PEAK_AFL_CONTROLLER_TYPE_AUTOFOCUS)
            self._af.SetMode(afl.PEAK_AFL_CONTROLLER_AUTOMODE_CONTINUOUS)
            # self.afl_mgr.AddController(self._af)

            # Optional: brightness controller (OFF by default on your setup)
            if self._use_brightness:
                self._bc = self.afl_mgr.CreateController(afl.PEAK_AFL_CONTROLLER_TYPE_BRIGHTNESS)
                self.afl_mgr.AddController(self._bc)
                print("[Camera] AFL Brightness controller added (USE_AFL_BRIGHTNESS=1).")
            else:
                self._bc = None
                print("[Camera] AFL Brightness controller disabled (recommended on your setup).")

            # Skip frames (helps first-frame blur)
            for ctrl, name in [(self._af, "AF")] + ([(self._bc, "BC")] if self._bc else []):
                try:
                    if ctrl and ctrl.IsSkipFramesSupported():
                        r = ctrl.GetSkipFramesRange()
                        val = 2
                        try:
                            val = int(max(getattr(r, "min", 0), min(val, getattr(r, "max", val))))
                        except Exception:
                            pass
                        ctrl.SetSkipFrames(val)
                        print(f"[Camera] AFL {name}: SetSkipFrames({val}) OK")
                except Exception as e:
                    print(f"[Camera] AFL {name}: SetSkipFrames failed: {e}")

            # Focus search limit
            try:
                if self._af and self._af.IsLimitSupported():
                    lim = self._af.GetDefaultLimit()
                    lim.min = int(self.focus_min)
                    lim.max = int(self.focus_max)
                    self._af.SetLimit(lim)
                    got = self._af.GetLimit()
                    print(f"[Camera] AFL focus limit set -> {getattr(got,'min',None)} .. {getattr(got,'max',None)}")
            except Exception as e:
                print(f"[Camera] Focus limit not set: {e}")

            self._afl_enabled = True
            print("[Camera] AFL configured (AF enabled).")

        except Exception as e:
            self.afl_mgr = None
            self._af = None
            self._bc = None
            self._afl_enabled = False
            print(f"[Camera] AFL configuration skipped: {e}")

    # -------------------------
    # Frame acquisition helpers
    # -------------------------

    def _grab_ipl_and_gray(self, timeout_ms: int = 5000):
        buf = self.ds.WaitForFinishedBuffer(timeout_ms)

        img = ipl_ext.BufferToImage(buf)

        # Force Mono8 for consistent stats/saving
        try:
            img = img.ConvertTo(ipl.PixelFormatName_Mono8)
        except Exception:
            pass

        try:
            gray = img.get_numpy_2D()
        except Exception:
            gray = img.get_numpy_3D()[:, :, 0]

        # IMPORTANT: do NOT queue here
        return buf, img, np.array(gray, copy=True)

    def _read_state(self) -> Dict[str, Optional[float]]:
        def read(node):
            try:
                v = _node_value(node)
                return float(v) if v is not None else None
            except Exception:
                return None

        return {
            "focus": read(self._focus_node),
            "exposure": read(self._exposure_node),
            "gain": read(self._gain_node),
        }

    def _frame_stats(self, gray: np.ndarray, roi_frac: float = 0.25) -> FrameStats:
        mean = float(gray.mean())
        lo, hi = clip_fractions(gray)
        sharp = sharpness_mean_grid(gray, frac=roi_frac)
        return FrameStats(mean=mean, lo_clip=lo, hi_clip=hi, sharp=sharp, mn=int(gray.min()), mx=int(gray.max()))

    def _is_invalid_frame(self, st: FrameStats) -> bool:
        # Reject “all black/invalid” frames
        if st.mean <= 0.5:
            return True
        if st.mx <= 2:
            return True
        return False

    def _soft_restart_stream(self, tag: str):
        # This is the “get out of black-frame jail” button.
        print(f"[Camera] [{tag}] Soft restart stream due to invalid frame streak...")
        try:
            _try_execute(self.nodemap, "AcquisitionStop")
        except Exception:
            pass
        try:
            if self.ds is not None:
                try:
                    self.ds.StopAcquisition()
                except Exception:
                    pass
                # Re-queue existing buffers
                try:
                    for b in self._buffers:
                        try:
                            self.ds.QueueBuffer(b)
                        except Exception:
                            pass
                except Exception:
                    pass
                try:
                    self.ds.StartAcquisition()
                except Exception:
                    pass
        except Exception:
            pass
        try:
            _try_execute(self.nodemap, "AcquisitionStart")
        except Exception:
            pass

        # Re-apply manual exp/gain after restart
        self._force_manual_exposure_gain(tag=f"{tag}/RESTART")

        self._invalid_streak = 0

    # -------------------------
    # Health check
    # -------------------------

    def health_check(self) -> None:
        print("[Camera][HC] ---- health check ----")
        state = self._read_state()
        print(f"[Camera][HC] Nodes: focus={'present' if self._focus_node else 'missing'}, "
            f"exposure={'present' if self._exposure_node else 'missing'}, "
            f"gain={'present' if self._gain_node else 'missing'}")
        print(f"[Camera][HC] State: focus={state['focus']} exp={state['exposure']} gain={state['gain']}")
        print(f"[Camera][HC] AFL enabled: {self._afl_enabled} (brightness={'on' if self._bc else 'off'})")

        try:
            # IMPORTANT: software trigger if enabled
            self._trigger_software()

            buf, ipl_img, frame = self._grab_ipl_and_gray(timeout_ms=5000)
            try:
                st = self._frame_stats(frame, roi_frac=0.25)
                print(f"[Camera][HC] Frame: shape={frame.shape} min={st.mn} max={st.mx} "
                    f"mean={st.mean:.1f} lo={st.lo_clip:.3f} hi={st.hi_clip:.3f} sharp={st.sharp:.1f}")
            finally:
                self.ds.QueueBuffer(buf)
        except Exception as e:
            print(f"[Camera][HC] Frame grab failed: {e}")

        print("[Camera][HC] ----------------------")

    # -------------------------
    # AFL run + capture (AF only by default)
    # -------------------------

    def run_afl_once(
        self,
        tag: str,
        frames: int = 30,
        settle_frames: int = 6,
        roi_frac: float = 0.25,
    ) -> np.ndarray:
        # We allow running even if AFL is off: you’ll still get a capture (manual exp/gain).
        use_afl = (self._afl_enabled and self.afl_mgr is not None and self._af is not None)

        # Always enforce manual exp/gain right before capture
        self._force_manual_exposure_gain(tag=f"{tag}/PRE")

        best = None
        best_st = None

        # Main loop
        for i in range(frames):
            # trigger one frame
            self._trigger_software()

            buf, ipl_img, gray = self._grab_ipl_and_gray(timeout_ms=5000)
            try:
                if use_afl:
                    try:
                        self.afl_mgr.Process(ipl_img)
                    except Exception as e:
                        print(f"[Camera] [{tag}] AFL Process failed: {e}")

                st = self._frame_stats(gray, roi_frac=roi_frac)
                state = self._read_state()

                if self._is_invalid_frame(st):
                    self._invalid_streak += 1
                else:
                    self._invalid_streak = 0

                if self._invalid_streak >= 3:
                    self._soft_restart_stream(tag=f"{tag}/LOOP")
                    # continue after restart (buffer will still be requeued by finally)
                    continue

                if (best_st is None) or (st.sharp > best_st.sharp):
                    best = gray
                    best_st = st

                print(
                    f"[Camera] [{tag}] {i+1}/{frames}: "
                    f"focus={state['focus']} exp={state['exposure']} gain={state['gain']} "
                    f"mean={st.mean:.1f} lo={st.lo_clip:.3f} hi={st.hi_clip:.3f} "
                    f"sharp={st.sharp:.1f} best={best_st.sharp if best_st else -1:.1f}",
                    flush=True,
                )
            finally:
                self.ds.QueueBuffer(buf)

        # Settle frames (best-of settle)
        settle_best = None
        settle_best_st = None
        for j in range(settle_frames):
            self._trigger_software()

            buf, ipl_img, gray = self._grab_ipl_and_gray(timeout_ms=5000)
            try:
                if use_afl:
                    try:
                        self.afl_mgr.Process(ipl_img)
                    except Exception:
                        pass

                st = self._frame_stats(gray, roi_frac=roi_frac)

                if self._is_invalid_frame(st):
                    self._invalid_streak += 1
                    if self._invalid_streak >= 3:
                        self._soft_restart_stream(tag=f"{tag}/SETTLE")
                    continue
                else:
                    self._invalid_streak = 0

                if settle_best_st is None or st.sharp > settle_best_st.sharp:
                    settle_best = gray
                    settle_best_st = st

                state = self._read_state()
                print(
                    f"[Camera] [{tag}] SETTLE {j+1}/{settle_frames}: "
                    f"focus={state['focus']} exp={state['exposure']} gain={state['gain']} "
                    f"mean={st.mean:.1f} lo={st.lo_clip:.3f} hi={st.hi_clip:.3f} sharp={st.sharp:.1f}",
                    flush=True,
                )
            finally:
                self.ds.QueueBuffer(buf)

        chosen = settle_best if settle_best is not None else best
        if chosen is None:
            raise RuntimeError("No valid frames captured")

        print(
            f"[Camera] [{tag}] done (best_sharp={best_st.sharp if best_st else -1:.1f}, "
            f"best_settle={settle_best_st.sharp if settle_best_st else -1:.1f}).",
            flush=True,
        )
        return chosen

    def take_single_capture(self) -> str:
        capture_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        print(f"[Camera] CAPTURE START id={capture_id}", flush=True)

        gray = self.run_afl_once(tag=capture_id)

        st = self._frame_stats(gray, roi_frac=0.25)
        state = self._read_state()

        out_path = self.capture_dir / f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.jpg"
        imwrite(out_path, gray)

        print(
            f"[Camera] [{capture_id}] SAVE: focus={state['focus']} exp={state['exposure']} gain={state['gain']} "
            f"mean={st.mean:.1f} lo={st.lo_clip:.3f} hi={st.hi_clip:.3f} sharp={st.sharp:.1f}",
            flush=True,
        )
        print(f"[Camera] CAPTURE END id={capture_id} -> {out_path}", flush=True)
        return str(out_path)

    # -------------------------
    # Cleanup
    # -------------------------

    def close(self):
        try:
            _try_execute(self.nodemap, "AcquisitionStop")
        except Exception:
            pass

        try:
            if self.ds is not None:
                try:
                    self.ds.StopAcquisition()
                except Exception:
                    pass

                for b in self._buffers:
                    try:
                        self.ds.RevokeBuffer(b)
                    except Exception:
                        pass

                try:
                    self.ds.Close()
                except Exception:
                    pass
        except Exception:
            pass

        self.device = None
        self.ds = None
        self.nodemap = None
        self._buffers = []

        try:
            ids_peak.Library.Close()
        except Exception:
            pass

        print("[Camera] Cleanup complete.")


if __name__ == "__main__":
    cam = SingleCameraController()
    cam.initialize()
    cam.health_check()
    cam.take_single_capture()
    cam.close()
