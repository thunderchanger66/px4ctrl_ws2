#!/usr/bin/env python3
"""Run isolated multi-point SITL validation with full stack restarts."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


ROOT = Path("/home/thunder/GRA_1")
WS = ROOT / "px4ctrl_ws2"
OUT = WS / "test/experiments/2026-11-13_five_point_dual_localization_p04_3_0"
OBSERVER = WS / "test/experiments/benchmark_observer.py"
LANDER = WS / "test/experiments/land_vehicle.py"
PX4_LAUNCH = ROOT / "mid360_ws/src/mid360_gazebo_harmonic/launch/mid360_posix_sitl.launch.py"
PX4_PARAMS = WS / "src/px4ctrl/config/px4ctrl_bringup.yaml"

CASES = [
    ("P01_2_0_0p5", 2.0, 0.0, 0.5),
    ("P02_2_m0p5_0p5", 2.0, -0.5, 0.5),
    ("P03_4_m4_0p5", 4.0, -4.0, 0.5),
    ("P04_3_0_0p5", 3.0, 0.0, 0.5),
    ("P05_2_0_1p5", 2.0, 0.0, 1.5),
]

SOURCE = (
    "source /opt/ros/jazzy/setup.bash && "
    "source /home/thunder/GRA_1/ws_livox/install/setup.bash && "
    "source /home/thunder/GRA_1/mid360_ws/install/setup.bash && "
    "source /home/thunder/GRA_1/fastlio2_pgo_ws/install/setup.bash && "
    "source /home/thunder/GRA_1/px4ctrl_ws2/install/setup.bash && "
    "export PX4_DIR=/home/thunder/PX4-Autopilot GRA_WORKSPACE=/home/thunder/GRA_1 && "
)


def start(command, logfile, cwd):
    stream = logfile.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["bash", "-lc", command], cwd=cwd, stdout=stream,
        stderr=subprocess.STDOUT, start_new_session=True,
    )
    return process, stream


def run_command(command, logfile, cwd, timeout):
    with logfile.open("w", encoding="utf-8") as stream:
        return subprocess.run(
            ["bash", "-lc", command], cwd=cwd, stdout=stream,
            stderr=subprocess.STDOUT, timeout=timeout, check=False,
        ).returncode


def wait_ros_topic(topic, timeout=90.0):
    deadline = time.monotonic() + timeout
    command = SOURCE + f"timeout 4 ros2 topic echo {topic} --once >/dev/null 2>&1"
    while time.monotonic() < deadline:
        probe = subprocess.run(["bash", "-lc", command], cwd=WS,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if probe.returncode == 0:
            return True
        time.sleep(1.0)
    return False


def wait_ego_goal_subscription(timeout=90.0):
    deadline = time.monotonic() + timeout
    command = SOURCE + "ros2 topic info /goal_pose"
    while time.monotonic() < deadline:
        probe = subprocess.run(["bash", "-lc", command], cwd=WS,
                               capture_output=True, text=True, check=False)
        if probe.returncode == 0:
            for line in probe.stdout.splitlines():
                if line.strip().startswith("Subscription count:"):
                    try:
                        if int(line.split(":", 1)[1].strip()) > 0:
                            return True
                    except ValueError:
                        pass
        time.sleep(1.0)
    return False


def stop(process, stream, timeout=25.0):
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=5.0)
    stream.close()


def one_trial(case):
    name, x, y, z = case
    trial_dir = OUT / name
    trial_dir.mkdir(parents=True, exist_ok=True)
    processes = []
    landing_done = False
    result = {
        "name": name, "goal_lidar_m": [x, y, z],
        "ego": {"max_vel_mps": 1.0, "max_acc_mps2": 1.0, "max_jerk_mps3": 3.0,
                "obstacle_inflation_m": 0.45, "collision_clearance_m": 0.65},
        "px4ctrl_profile": str(PX4_PARAMS),
        "outcome": "not_started",
    }
    try:
        sim, sim_log = start(
            SOURCE + f"ros2 launch {PX4_LAUNCH} gui:=false",
            trial_dir / "px4_gazebo.log", ROOT,
        )
        processes.append((sim, sim_log))
        if not wait_ros_topic("/clock", 120.0):
            result["outcome"] = "failed_no_clock"
            return result

        pgo, pgo_log = start(
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/thunder/GRA_1/ws_livox/install/setup.bash && "
            "source /home/thunder/GRA_1/fastlio2_pgo_ws/install/setup.bash && "
            "ros2 launch pgo pgo_launch.py use_sim_time:=true",
            trial_dir / "fastlio_pgo.log", ROOT / "fastlio2_pgo_ws",
        )
        processes.append((pgo, pgo_log))
        if not wait_ros_topic("/fastlio2/lio_odom", 90.0):
            result["outcome"] = "failed_no_fastlio_odom"
            return result

        bringup_args = (
            f"ros2 launch px4ctrl bringup.launch.py start_agent:=true start_rviz:=false "
            "start_takeoff:=false "
            f"px4ctrl_params:={PX4_PARAMS} ego_max_vel:=1.0 ego_max_acc:=1.0 "
            "ego_max_jerk:=3.0 ego_lambda_smooth:=2.0 ego_planning_horizon:=7.5 "
            "ego_obstacle_inflation:=0.45 ego_collision_clearance:=0.65"
        )
        bringup, bringup_log = start(SOURCE + bringup_args,
                                     trial_dir / "bringup.log", WS)
        processes.append((bringup, bringup_log))
        if not wait_ros_topic("/px4ctrl/status", 90.0):
            result["outcome"] = "failed_no_controller"
            return result
        if not wait_ego_goal_subscription(90.0):
            result["outcome"] = "failed_no_ego_goal_subscription"
            return result

        observer_cmd = (
            SOURCE + f"python3 {OBSERVER} --csv {trial_dir / 'samples.csv'} "
            f"--goal-x {x} --goal-y {y} --goal-z {z} --timeout 60 "
            "--startup-timeout 600"
        )
        observer, observer_log = start(observer_cmd, trial_dir / "observer.log", WS)
        processes.append((observer, observer_log))
        time.sleep(2.0)

        takeoff_rc = run_command(
            SOURCE + "ros2 run px4ctrl takeoff --altitude 0.5 --wait-timeout 45 --require-ev-fusion",
            trial_dir / "takeoff.log", WS, 90.0,
        )
        result["takeoff_exit_code"] = takeoff_rc
        if takeoff_rc != 0:
            result["outcome"] = "failed_takeoff"
        else:
            goal_rc = run_command(
                SOURCE + f"ros2 run px4ctrl manual_goal {x} {y} {z} --auto-yaw --wait-timeout 30",
                trial_dir / "manual_goal.log", WS, 60.0,
            )
            result["goal_command_exit_code"] = goal_rc
            retry_exit_codes = []
            observer_log_path = trial_dir / "observer.log"
            bringup_log_path = trial_dir / "bringup.log"
            for attempt in range(2):
                settle_deadline = time.monotonic() + 8.0
                while time.monotonic() < settle_deadline:
                    observer_text = observer_log_path.read_text(encoding="utf-8", errors="replace")
                    bringup_text = bringup_log_path.read_text(encoding="utf-8", errors="replace")
                    if "Received first EGO B-spline" in observer_text:
                        break
                    if "Rejecting goal inside an occupied" in bringup_text or \
                       "Rejecting unreachable waypoint" in bringup_text:
                        break
                    time.sleep(0.25)
                observer_text = observer_log_path.read_text(encoding="utf-8", errors="replace")
                bringup_text = bringup_log_path.read_text(encoding="utf-8", errors="replace")
                if ("Received first EGO B-spline" in observer_text or
                        "Rejecting goal inside an occupied" in bringup_text or
                        "Rejecting unreachable waypoint" in bringup_text):
                    break
                retry_rc = run_command(
                    SOURCE + f"ros2 run px4ctrl manual_goal {x} {y} {z} --auto-yaw --wait-timeout 30",
                    trial_dir / f"manual_goal_retry_{attempt + 1}.log", WS, 60.0,
                )
                retry_exit_codes.append(retry_rc)
            result["goal_retry_exit_codes"] = retry_exit_codes
            observer_timeout = observer.wait(timeout=180.0)
            result["observer_exit_code"] = observer_timeout
            summary_path = trial_dir / "samples.json"
            if summary_path.exists():
                result.update(json.loads(summary_path.read_text(encoding="utf-8")))
            else:
                result["outcome"] = "failed_no_observer_summary"

        land_rc = run_command(
            SOURCE + f"python3 {LANDER} --timeout 90",
            trial_dir / "landing.log", WS, 110.0,
        )
        result["landing_exit_code"] = land_rc
        result["landing_confirmed"] = land_rc == 0
        landing_done = True
        if result.get("trial_valid") and land_rc == 0:
            result["outcome"] = "arrived"
        elif result.get("trial_valid"):
            result["outcome"] = "arrived_landing_unconfirmed"
        elif result.get("outcome") == "not_started":
            result["outcome"] = result.get("result", "observer_invalid")
        bringup_text = (trial_dir / "bringup.log").read_text(encoding="utf-8", errors="replace")
        if result.get("ego_bspline_messages", 0) == 0 and \
           ("Rejecting goal inside an occupied" in bringup_text or
            "Rejecting unreachable waypoint" in bringup_text):
            result["outcome"] = "goal_rejected_in_occupied_map"
        elif result.get("ego_bspline_messages", 0) == 0 and result.get("takeoff_exit_code") == 0:
            result["outcome"] = "goal_not_processed_no_trajectory"
        result["clearance_gate_pass"] = (
            result.get("inflated_map_clearance_min_m") is not None and
            result["inflated_map_clearance_min_m"] >= 0.50
        )
        if result["outcome"] == "arrived" and not result["clearance_gate_pass"]:
            result["outcome"] = "arrived_safety_gate_failed"
    except subprocess.TimeoutExpired:
        result["outcome"] = "command_timeout"
    except Exception as exc:
        result["outcome"] = "exception"
        result["error"] = repr(exc)
    finally:
        if processes and not landing_done:
            try:
                result["landing_exit_code"] = run_command(
                    SOURCE + f"python3 {LANDER} --timeout 90",
                    trial_dir / "landing_fallback.log", WS, 110.0,
                )
                result["landing_confirmed"] = result["landing_exit_code"] == 0
            except Exception as exc:
                result["landing_confirmed"] = False
                result["landing_error"] = repr(exc)
        for process, stream in reversed(processes):
            stop(process, stream)
        (trial_dir / "result.json").write_text(
            json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return result


def main():
    global OUT
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=OUT)
    parser.add_argument("--cases", help="Comma-separated case names; default runs every case")
    parser.add_argument("--point", help="One custom point as NAME,X,Y,Z")
    args = parser.parse_args()
    OUT = args.output.resolve()
    OUT.mkdir(parents=True, exist_ok=True)
    selected = CASES
    if args.point:
        try:
            name, x, y, z = args.point.split(",")
            selected = [(name, float(x), float(y), float(z))]
        except ValueError:
            parser.error("--point must be NAME,X,Y,Z")
    if args.cases:
        if args.point:
            parser.error("--point and --cases cannot be used together")
        wanted = set(args.cases.split(","))
        unknown = wanted - {case[0] for case in CASES}
        if unknown:
            parser.error("unknown case name(s): " + ", ".join(sorted(unknown)))
        selected = [case for case in CASES if case[0] in wanted]
    suite = []
    for index, case in enumerate(selected):
        result = one_trial(case)
        suite.append(result)
        print(json.dumps({key: result.get(key) for key in (
            "name", "outcome", "arrival_time_s", "track_error_3d_rmse_m",
            "track_error_3d_p95_m", "goal_error_final_m",
            "px4_local_ned_tracking_error_rmse_m",
            "px4_local_ned_tracking_error_p95_m",
            "px4_local_ned_goal_error_final_m", "speed_peak_mps",
            "command_accel_peak_mps2", "inflated_map_clearance_min_m",
            "failsafe_samples", "landing_confirmed", "clearance_gate_pass",
        )}, allow_nan=False), flush=True)
        if not result.get("landing_confirmed", False):
            for skipped in selected[index + 1:]:
                suite.append({
                    "name": skipped[0],
                    "goal_lidar_m": [skipped[1], skipped[2], skipped[3]],
                    "outcome": "not_run_previous_landing_unconfirmed",
                    "landing_confirmed": False,
                })
            break
    summary = {"cases": suite,
               "complete": len(suite) == len(selected) and
                           all(case.get("outcome") != "not_run_previous_landing_unconfirmed"
                               for case in suite),
               "selected_case_names": [case[0] for case in selected]}
    (OUT / "suite_results.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
