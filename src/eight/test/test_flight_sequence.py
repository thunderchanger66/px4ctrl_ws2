"""Check the automatic mission handoff without starting a simulator."""

from types import SimpleNamespace
import unittest

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

from eight.eight_node import EightNode
from eight.sitl_gcs import _float_as_int, _int_as_float


class FakeSequence:
    def __init__(self):
        self.stage = "waiting_for_px4"
        self.stage_start_ns = 0
        self.command_denial = ""
        self.vehicle_status = SimpleNamespace(
            arming_state=VehicleStatus.ARMING_STATE_DISARMED,
            nav_state=VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
            failsafe=False,
        )
        self.vehicle_status_time_ns = 1
        self.local_position = SimpleNamespace(x=0.0, y=0.0, z=0.0, heading=0.0)
        self.controller_status = ""
        self.controller_status_time_ns = 1
        self.takeoff_timeout = 30.0
        self.takeoff_altitude = 2.0
        self.takeover_timeout = 20.0
        self.owns_flight = False
        self.commands = []
        self.started = False
        self.aborted = ""
        self.get_logger = lambda: SimpleNamespace(info=lambda message: None, error=lambda message: None)

    def _fresh(self, *args):
        return True

    def _preflight_ready(self, now_ns):
        return True

    def _stage(self, name, now_ns):
        self.stage = name
        self.stage_start_ns = now_ns

    def _send_command(self, command, *params):
        self.commands.append((command, params))

    def _ready(self, now_ns):
        return self.controller_status == "hover_airborne_takeover_offboard_ready"

    def _begin(self, now_ns):
        self.started = True

    def _abort(self, reason):
        self.aborted = reason

    def _diagnose_takeover(self):
        pass

    def _publish_takeoff_setpoint(self, now_ns):
        pass


class FlightSequenceTests(unittest.TestCase):
    def test_takeoff_arm_offboard_stops_before_trajectory(self):
        flight = FakeSequence()
        advance = lambda t: EightNode._advance_startup(flight, t)
        advance(1)
        self.assertEqual(flight.stage, "prestream")
        self.assertEqual(flight.takeoff_target, (0.0, 0.0, -2.0))
        advance(1_500_000_002)
        self.assertEqual(flight.stage, "offboard")
        advance(1_500_000_003)
        self.assertEqual(flight.commands[-1][0], VehicleCommand.VEHICLE_CMD_DO_SET_MODE)
        flight.vehicle_status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        advance(1_500_000_004)
        self.assertEqual(flight.stage, "arming")
        advance(1_500_000_005)
        self.assertEqual(flight.commands[-1][0], VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
        flight.vehicle_status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        advance(1_500_000_006)
        self.assertTrue(flight.owns_flight)
        self.assertEqual(flight.stage, "airborne")
        advance(1_500_000_007)
        self.assertEqual(flight.stage, "airborne")
        flight.controller_status = "hover_airborne_takeover_prestream"
        advance(1_500_000_008)
        self.assertEqual(flight.stage, "handoff")
        flight.controller_status = "hover_airborne_takeover_offboard_ready"
        advance(1_500_000_009)
        self.assertFalse(flight.started)
        self.assertEqual(flight.stage, "ready")

    def test_trajectory_phase_requires_offboard_controller(self):
        flight = FakeSequence()
        flight.stage = "waiting_for_controller"
        flight.process_start_wall = __import__("time").monotonic()
        flight.startup_timeout = 45.0
        flight._now_ns = lambda: 100
        EightNode._tick(flight)
        self.assertFalse(flight.started)
        flight.controller_status = "hover_airborne_takeover_offboard_ready"
        EightNode._tick(flight)
        self.assertTrue(flight.started)
        self.assertEqual(flight.stage, "tracking")

    def test_offboard_handoff_times_out_if_px4_rejects_mode(self):
        flight = FakeSequence()
        flight.stage = "offboard"
        flight.stage_start_ns = 1
        EightNode._advance_startup(flight, 21_000_000_002)
        self.assertEqual(flight.aborted, "PX4 did not enter position Offboard after prestream")
        self.assertFalse(flight.started)

    def test_position_setpoint_is_ned_and_permission_waits_for_target(self):
        flight = FakeSequence()
        flight.stage = "airborne"
        flight.takeoff_target = (1.0, -2.0, -2.0)
        flight.takeoff_heading = 0.4
        flight.local_position = SimpleNamespace(
            timestamp=100, dead_reckoning=False, xy_valid=True, z_valid=True,
            v_xy_valid=True, v_z_valid=True, x=1.0, y=-2.0, z=-2.0,
            vx=0.0, vy=0.0, vz=0.0, heading=0.4)
        flight.local_position_time_ns = 100
        flight.land_detected = SimpleNamespace(landed=False, maybe_landed=False, ground_contact=False)
        flight.land_detected_time_ns = 100
        flight.land_detected_timeout = 2.0
        flight.takeover_permission_sent = False
        flight.takeoff_stable_since_ns = None
        mode, position, permission = [], [], []
        flight.mode_pub = SimpleNamespace(publish=mode.append)
        flight.position_setpoint_pub = SimpleNamespace(publish=position.append)
        flight.takeover_permission_pub = SimpleNamespace(publish=permission.append)
        flight._at_takeoff_target = lambda now_ns: EightNode._at_takeoff_target(flight, now_ns)
        flight._fresh = lambda last, now, max_age_sec=0.5: EightNode._fresh(last, now, max_age_sec)
        EightNode._publish_takeoff_setpoint(flight, 100)
        self.assertIsInstance(mode[-1], OffboardControlMode)
        self.assertTrue(mode[-1].position)
        self.assertIsInstance(position[-1], TrajectorySetpoint)
        self.assertEqual(list(position[-1].position), [1.0, -2.0, -2.0])
        self.assertFalse(permission[-1].data)
        flight.local_position_time_ns = 600_000_101
        EightNode._publish_takeoff_setpoint(flight, 600_000_101)
        self.assertTrue(permission[-1].data)

    def test_reject_already_armed_vehicle(self):
        flight = FakeSequence()
        flight.vehicle_status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        flight.get_logger = lambda: SimpleNamespace(error=lambda message: None)
        EightNode._advance_startup(flight, 1)
        self.assertEqual(flight.stage, "done")
        self.assertFalse(flight.commands)

    def test_px4_integer_parameter_bitcast(self):
        for value in (0, 4, 7, 31):
            self.assertEqual(_float_as_int(_int_as_float(value)), value)

    def test_landing_requests_px4_land_until_disarmed(self):
        flight = FakeSequence()
        flight.stage = "landing"
        flight.vehicle_status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        flight._now_ns = lambda: 100
        EightNode._tick(flight)
        self.assertEqual(flight.commands[-1][0], VehicleCommand.VEHICLE_CMD_NAV_LAND)
        flight.vehicle_status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        EightNode._tick(flight)
        self.assertEqual(flight.stage, "done")
