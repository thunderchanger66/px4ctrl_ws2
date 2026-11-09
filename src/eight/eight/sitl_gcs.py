"""Provide a local GCS heartbeat and no-RC Offboard exception for X500 SITL."""

import logging
import signal
import struct
import time

from pymavlink import mavutil


LOG = logging.getLogger("eight.sitl_gcs")
PARAMETER = b"COM_RCL_EXCEPT"
NO_RC_OFFBOARD = 4


def _int_as_float(value: int) -> float:
    return struct.unpack("<f", struct.pack("<i", value))[0]


def _float_as_int(value: float) -> int:
    return struct.unpack("<i", struct.pack("<f", value))[0]


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    connection = mavutil.mavlink_connection(
        "udpin:0.0.0.0:14550", source_system=255,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
    )
    running = True

    def stop(_signal, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    last_heartbeat = 0.0
    last_request = 0.0
    original = None
    configured = False
    while running:
        now = time.monotonic()
        if now - last_heartbeat >= 1.0:
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            last_heartbeat = now
        message = connection.recv_match(blocking=False)
        if message and message.get_type() == "HEARTBEAT" and message.get_srcSystem() != 255:
            connection.target_system = message.get_srcSystem()
            connection.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        elif message and message.get_type() == "PARAM_VALUE" and \
                (message.param_id.decode() if isinstance(message.param_id, bytes)
                 else message.param_id).rstrip("\x00") == PARAMETER.decode():
            value = _float_as_int(message.param_value)
            if original is None:
                original = value
            if value & NO_RC_OFFBOARD:
                if not configured:
                    LOG.info("PX4 COM_RCL_EXCEPT=%d allows no-RC Offboard SITL", value)
                configured = True
            elif connection.target_system:
                connection.mav.param_set_send(
                    connection.target_system, connection.target_component,
                    PARAMETER, _int_as_float(value | NO_RC_OFFBOARD),
                    mavutil.mavlink.MAV_PARAM_TYPE_INT32,
                )
        if not configured and connection.target_system and now - last_request >= 2.0:
            connection.mav.param_request_read_send(
                connection.target_system, connection.target_component, PARAMETER, -1,
            )
            last_request = now
        time.sleep(0.05)
    if original is not None and not (original & NO_RC_OFFBOARD) and connection.target_system:
        connection.mav.param_set_send(
            connection.target_system, connection.target_component, PARAMETER,
            _int_as_float(original), mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        time.sleep(0.2)
    connection.close()


if __name__ == "__main__":
    main()
