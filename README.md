# Spot CORE I/O IDS Single Camera Capture

Boston Dynamics Spot CORE I/O extension for triggering a single IDS camera capture from a Spot RemoteMissionService node.

The service initializes one IDS USB camera at startup, registers a RemoteMissionService with Spot, and exposes a mission command that captures a focused still image. Captures are written to a configurable directory on CORE I/O, normally under `/data/captures`.

## Repository Layout

```text
.
├── src/
│   ├── robot_command_mission_service.py   # Spot RemoteMissionService entrypoint
│   └── single_camera_controller.py        # IDS camera setup, AFL focus, capture logic
├── Dockerfile.l4t                         # ARM64 Jetson/CORE I/O container image
├── docker-compose.yml                     # CORE I/O extension service definition
├── manifest.json                          # Spot extension manifest
├── 99-ids-usb.rules                       # udev rule for IDS USB cameras
└── ids-peak-with-ueyetl_*.tar.gz          # IDS peak runtime archive for ARM64
```

Generated extension artifacts may also be present:

```text
ids_single_camera.spx
ids_single_camera.tgz
```

These files are large and are ignored by `.gitignore`; commit them only if you intentionally want to publish a built extension package.

## What It Does

- Runs on Spot CORE I/O as a Docker extension.
- Uses IDS peak, IDS peak IPL, and IDS peak AFL Python bindings.
- Opens the first available IDS camera, or a specific serial from `CAMERA_SERIAL`.
- Uses software trigger mode for single-frame capture.
- Forces manual exposure/gain settings to avoid black-frame captures.
- Runs autofocus processing and selects the sharpest frame.
- Saves captured images as `.jpg`.
- Registers with Spot as `single-camera-capture-service`.

Remote mission command:

```text
take_picture
```

The mission command returns success after the image has been saved.

## Hardware Requirements

- Boston Dynamics Spot with CORE I/O
- IDS USB camera connected to CORE I/O
- IDS peak runtime package for ARM64:

```text
ids-peak-with-ueyetl_2.17.1.0-559_arm64.tar.gz
```

- Spot network access from CORE I/O

The Docker image targets NVIDIA L4T/Jetson:

```dockerfile
nvcr.io/nvidia/l4t-jetpack:r35.3.1
```

## Configuration

### Spot and Service Settings

`docker-compose.yml` sets the main Spot connection values:

```yaml
SPOT_HOSTNAME: "192.168.50.3"
SPOT_USERNAME: "user"
SPOT_PASSWORD: "your-password"
CAPTURE_DIR: "/data/captures"
```

Update these before building or deploying to your robot. Do not commit real robot credentials to a public repository.

The service is started with:

```yaml
python3 /app/robot_command_mission_service.py --host-ip 192.168.50.5 --port 21222 192.168.50.3
```

Adjust:

- `--host-ip`: CORE I/O IP reachable by Spot
- `--port`: RemoteMissionService gRPC port
- final positional IP: Spot hostname/IP

### Camera Settings

The controller reads these optional environment variables:

```text
CAMERA_SERIAL          IDS camera serial to use. Empty means first detected camera.
CAPTURE_DIR            Directory where images are saved.
EXPOSURE_US            Manual exposure time in microseconds.
EXPOSURE_MIN_US        Minimum exposure clamp.
EXPOSURE_MAX_US        Maximum exposure clamp.
GAIN                   Manual gain value.
GAIN_MIN               Minimum gain clamp.
GAIN_MAX               Maximum gain clamp.
FOCUS_MIN              Autofocus lower focus limit.
FOCUS_MAX              Autofocus upper focus limit.
IDS_NUM_BUFFERS        Number of IDS stream buffers.
USE_AFL_BRIGHTNESS     Set to 1 to enable AFL brightness controller.
```

By default, AFL brightness is disabled because manual exposure/gain is more stable for this setup.

## Build the Docker Image

From the repository root:

```bash
docker build --platform linux/arm64 -f Dockerfile.l4t -t ids_single_camera:1.0 .
```

The image must be ARM64 for CORE I/O. If it is built for the wrong architecture, the extension can fail with:

```text
exec /usr/bin/sh: exec format error
```

## Package the CORE I/O Extension

Create the image archive:

```bash
docker save ids_single_camera:1.0 | gzip > ids_single_camera.tgz
```

Create the Spot extension package:

```bash
tar -czf ids_single_camera.spx manifest.json docker-compose.yml 99-ids-usb.rules ids_single_camera.tgz
```

Deploy `ids_single_camera.spx` through the Spot extension interface.

## Run Locally on CORE I/O

If you are testing directly on CORE I/O with Docker Compose:

```bash
docker compose up
```

The compose file:

- Uses `network_mode: host`
- Maps USB devices through `/dev/bus/usb`
- Mounts `/data` into the container
- Writes captures to `/data/captures`

## Run the Camera Controller Directly

For camera-only testing in an environment with IDS libraries installed:

```bash
python3 src/single_camera_controller.py
```

This initializes the camera, performs a health check, captures one image, and closes the camera.

## RemoteMissionService Details

Entrypoint:

```text
src/robot_command_mission_service.py
```

Directory registration:

```text
single-camera-capture-service
```

Service type:

```text
bosdyn.api.mission.RemoteMissionService
```

The service requires a Spot body lease. During `Tick`, it retains the lease asynchronously and triggers capture when the mission parameter `command` is `take_picture`.

## Captures

Captured files are named like:

```text
capture_YYYYMMDD_HHMMSS_mmm.jpg
```

Default container output:

```text
/data/captures
```

The compose file maps host `/data` to container `/data`, so images should persist on CORE I/O under:

```text
/data/captures
```

## Troubleshooting

### No IDS camera detected

Check USB visibility:

```bash
lsusb
```

The udev rule targets IDS vendor ID `1409`:

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="1409", MODE="0666", GROUP="plugdev"
```

Confirm the camera is connected to CORE I/O and visible inside the container through `/dev/bus/usb`.

### Black or invalid frames

The controller forces manual exposure and gain before capture. Tune:

```text
EXPOSURE_US
GAIN
FOCUS_MIN
FOCUS_MAX
```

The controller also soft-restarts the stream after repeated invalid frames.

### Service cannot register with Spot

Check:

- `SPOT_HOSTNAME`
- `SPOT_USERNAME`
- `SPOT_PASSWORD`
- `--host-ip`
- Robot and CORE I/O network connectivity

### Wrong image architecture

Verify the image:

```bash
docker image inspect ids_single_camera:1.0 --format '{{.Architecture}} {{.Os}}'
```

Expected:

```text
arm64 linux
```

## Security Notes

- Do not commit real Spot passwords.
- Consider moving credentials out of `docker-compose.yml` into an env file before publishing.
- Large vendor/runtime archives and generated extension packages should be distributed intentionally, not committed by default.
