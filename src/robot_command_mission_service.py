# robot_command_mission_service.py

import logging
import os
import random
import string
import threading

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import header_pb2
from bosdyn.api.mission import remote_pb2, remote_service_pb2_grpc
from bosdyn.client import time_sync
from bosdyn.client.directory_registration import (
    DirectoryRegistrationClient,
    DirectoryRegistrationKeepAlive,
)
from bosdyn.client.lease import Lease, LeaseClient
from bosdyn.client.server_util import GrpcServiceRunner, ResponseContext
from bosdyn.client.service_customization_helpers import (
    create_value_validator,
    make_dict_child_spec,
    make_dict_param_spec,
    make_string_param_spec,
    make_user_interface_info,
    validate_dict_spec,
)
from bosdyn.client.util import setup_logging

from single_camera_controller import SingleCameraController

DIRECTORY_NAME = "single-camera-capture-service"
AUTHORITY = "remote-mission"
SERVICE_TYPE = "bosdyn.api.mission.RemoteMissionService"

_LOGGER = logging.getLogger(__name__)

_COMMAND_KEY = "command"


class CaptureRemoteMissionServicer(
    remote_service_pb2_grpc.RemoteMissionServiceServicer
):
    """RemoteMissionService exposing a single 'take_picture' command."""

    RESOURCE = "body"

    def __init__(self, bosdyn_sdk_robot, camera: SingleCameraController, logger=None):
        self.lock = threading.Lock()
        self.logger = logger or _LOGGER
        self.bosdyn_sdk_robot = bosdyn_sdk_robot
        self.camera = camera
        self.sessions_by_id = {}
        self._used_session_ids = []

        # UI schema: just a 'command' enum
        command_param = make_string_param_spec(
            options=["take_picture", "noop"],
            default_value="noop",
            editable=True,
        )
        command_ui_info = make_user_interface_info(
            "Capture Command", "Trigger a single still capture via AFL."
        )

        dict_spec = make_dict_param_spec(
            {
                _COMMAND_KEY: make_dict_child_spec(command_param, command_ui_info),
            },
            is_hidden_by_default=False,
        )
        validate_dict_spec(dict_spec)
        self.custom_params = dict_spec

    # -------- leases / sessions --------
    def _get_unique_random_session_id(self):
        while True:
            sid = "".join([random.choice(string.ascii_letters) for _ in range(16)])
            if sid not in self._used_session_ids:
                return sid

    def _sublease_or_none(self, leases, response, error_code):
        matches = [l for l in leases if l.resource == self.RESOURCE]
        if len(matches) == 1:
            provided_lease = Lease(matches[0])
            return provided_lease.create_sublease()
        if not matches:
            response.status = error_code
            response.missing_lease_resources.append(self.RESOURCE)
            return None
        response.header.error.code = header_pb2.CommonError.CODE_INVALID_REQUEST
        response.header.error.message = (
            f"{len(matches)} leases on resource {self.RESOURCE}"
        )
        return None

    def EstablishSession(self, request, context):
        response = remote_pb2.EstablishSessionResponse()
        with ResponseContext(response, request):
            with self.lock:
                sublease = self._sublease_or_none(
                    request.leases,
                    response,
                    remote_pb2.EstablishSessionResponse.STATUS_MISSING_LEASES,
                )
                if sublease is None:
                    return response
                try:
                    self.bosdyn_sdk_robot.time_sync.wait_for_sync()
                except time_sync.TimedOutError:
                    response.header.error.code = (
                        header_pb2.CommonError.CODE_INTERNAL_SERVER_ERROR
                    )
                    response.header.error.message = (
                        "Failed to time sync with robot"
                    )
                    return response
                sid = self._get_unique_random_session_id()
                self.sessions_by_id[sid] = {}
                self._used_session_ids.append(sid)
                response.session_id = sid
                response.status = remote_pb2.EstablishSessionResponse.STATUS_OK
        return response

    def GetRemoteMissionServiceInfo(self, request, context):
        response = remote_pb2.GetRemoteMissionServiceInfoResponse()
        with ResponseContext(response, request):
            response.custom_params.CopyFrom(self.custom_params)
        return response

    # -------- core Tick/Stop --------
    def Tick(self, request, context):
        response = remote_pb2.TickResponse()
        with ResponseContext(response, request):
            with self.lock:
                if request.session_id not in self.sessions_by_id:
                    response.status = remote_pb2.TickResponse.STATUS_INVALID_SESSION_ID
                    return response

                sublease = self._sublease_or_none(
                    request.leases,
                    response,
                    remote_pb2.TickResponse.STATUS_MISSING_LEASES,
                )
                if sublease is None:
                    return response

                valid = create_value_validator(self.custom_params)(request.params)
                if valid is not None:
                    response.status = remote_pb2.TickResponse.STATUS_CUSTOM_PARAMS_ERROR
                    response.custom_param_error.CopyFrom(valid)
                    return response

                command = None
                if (
                    _COMMAND_KEY in request.params.values
                    and request.params.values[_COMMAND_KEY].WhichOneof("value")
                    == "string_value"
                ):
                    command = request.params.values[
                        _COMMAND_KEY
                    ].string_value.value

                # keep lease alive in background
                lease_client = self.bosdyn_sdk_robot.ensure_client(
                    LeaseClient.default_service_name
                )
                lease_client.retain_lease_async(sublease)

                if command == "take_picture":
                    try:
                        img_path = self.camera.take_single_capture()
                        self.logger.info(f"Captured still image: {img_path}")
                        response.status = remote_pb2.TickResponse.STATUS_SUCCESS
                    except Exception as e:
                        self.logger.exception("Capture failed")
                        response.status = remote_pb2.TickResponse.STATUS_FAILURE
                        response.header.error.code = (
                            header_pb2.CommonError.CODE_INTERNAL_SERVER_ERROR
                        )
                        response.header.error.message = str(e)
                    return response

                # 'noop' or unspecified → keep running
                response.status = remote_pb2.TickResponse.STATUS_RUNNING
        return response

    def Stop(self, request, context):
        response = remote_pb2.StopResponse()
        with ResponseContext(response, request):
            response.status = remote_pb2.StopResponse.STATUS_OK
        return response

    def TeardownSession(self, request, context):
        response = remote_pb2.TeardownSessionResponse()
        with ResponseContext(response, request):
            with self.lock:
                if request.session_id in self.sessions_by_id:
                    del self.sessions_by_id[request.session_id]
                    response.status = (
                        remote_pb2.TeardownSessionResponse.STATUS_OK
                    )
                else:
                    response.status = (
                        remote_pb2.TeardownSessionResponse.STATUS_INVALID_SESSION_ID
                    )
        return response


def run_service(bosdyn_sdk_robot, port, camera: SingleCameraController, logger=None):
    service_servicer = CaptureRemoteMissionServicer(
        bosdyn_sdk_robot, camera, logger=logger
    )
    return GrpcServiceRunner(
        service_servicer,
        remote_service_pb2_grpc.add_RemoteMissionServiceServicer_to_server,
        port,
        logger=logger,
    )


def main():
    import argparse
    import sys
    import os
    
    # CORE I/O extension mode: allow hostname to come from env rather than CLI.
    # bosdyn.client.util.add_base_arguments(parser) requires a positional "hostname".
    if len(sys.argv) == 1:  # no CLI args provided
        env_host = os.environ.get("SPOT_HOSTNAME") or os.environ.get("BOSDYN_HOSTNAME")
        if env_host:
            sys.argv.append(env_host)

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    bosdyn.client.util.add_service_endpoint_arguments(parser)
    options = parser.parse_args()

    setup_logging(options.verbose)

    # Create & authenticate robot (env creds recommended)
    sdk = bosdyn.client.create_standard_sdk("SingleCameraCaptureSDK")
    robot = sdk.create_robot(options.hostname)

    user = os.environ.get("SPOT_USERNAME") or os.environ.get("BOSDYN_CLIENT_USERNAME")
    pw = os.environ.get("SPOT_PASSWORD") or os.environ.get("BOSDYN_CLIENT_PASSWORD")

    if user and pw:
        robot.authenticate(user, pw)
    else:
        raise RuntimeError(
            "Set SPOT_USERNAME/SPOT_PASSWORD (or BOSDYN_CLIENT_USERNAME/BOSDYN_CLIENT_PASSWORD) in the environment."
        )

    # Initialize camera once at service startup
    camera = SingleCameraController()
    camera.initialize()

    # Start gRPC server
    service_runner = run_service(robot, options.port, camera=camera, logger=_LOGGER)

    # Register in directory
    dir_reg_client = robot.ensure_client(
        DirectoryRegistrationClient.default_service_name
    )
    keep_alive = DirectoryRegistrationKeepAlive(dir_reg_client, logger=_LOGGER)
    keep_alive.start(
        DIRECTORY_NAME,
        SERVICE_TYPE,
        AUTHORITY,
        options.host_ip,
        service_runner.port,
    )

    with keep_alive:
        try:
            service_runner.run_until_interrupt()
        finally:
            camera.close()


if __name__ == "__main__":
    main()
