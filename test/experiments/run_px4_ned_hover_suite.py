#!/usr/bin/env python3
"""Run five isolated SITL trials from a settled PX4 hover point."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


ROOT = Path("/home/thunder/GRA_1")
WS = Path(__file__).resolve().parents[2]
DEFAULT_OUT = WS / "test/experiments/2026-11-20_kv14_seven_experiments"
OBSERVER = WS / "test/experiments/px4_ned_hover_observer.py"
LANDER = WS / "test/experiments/land_vehicle.py"
PX4_LAUNCH = ROOT / "mid360_ws/src/mid360_gazebo_harmonic/launch/mid360_posix_sitl.launch.py"
PX4_PARAMS = WS / "src/px4ctrl/config/px4ctrl_bringup.yaml"

POINTS = [
    ("P01_2_0_0p5", 2.0, 0.0, 0.5),
    ("P02_2_m0p5_0p5", 2.0, -0.5, 0.5),
    ("P03_4_m4_0p5", 4.0, -4.0, 0.5),
    ("P04_3_0_0p5", 3.0, 0.0, 0.5),
    ("P05_2_0_1p5", 2.0, 0.0, 1.5),
    ("P06_10_0_0p5", 10.0, 0.0, 0.5),
]

EXPERIMENTS = [
    {"name": name, "goals": [[x, y, z]], "continuous": False}
    for name, x, y, z in POINTS
]
EXPERIMENTS.append({
    "name": "P07_continuous_all_six",
    "goals": [[x, y, z] for _, x, y, z in POINTS],
    "continuous": True,
})

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
    process = subprocess.Popen(["bash", "-lc", command], cwd=cwd,
        stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    return process, stream


def run_command(command, logfile, cwd, timeout):
    with logfile.open("w", encoding="utf-8") as stream:
        return subprocess.run(["bash", "-lc", command], cwd=cwd,
            stdout=stream, stderr=subprocess.STDOUT, timeout=timeout,
            check=False).returncode


def wait_topic(topic, timeout=90.0):
    deadline = time.monotonic() + timeout
    command = SOURCE + f"timeout 4 ros2 topic echo {topic} --once >/dev/null 2>&1"
    while time.monotonic() < deadline:
        if subprocess.run(["bash", "-lc", command], cwd=WS,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
            return True
        time.sleep(1.0)
    return False


def has_goal_subscriber(timeout=90.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        probe = subprocess.run(["bash", "-lc", SOURCE + "ros2 topic info /goal_pose"],
            cwd=WS, capture_output=True, text=True, check=False)
        if probe.returncode == 0:
            for line in probe.stdout.splitlines():
                if line.strip().startswith("Subscription count:"):
                    try:
                        if int(line.split(":", 1)[1]) > 0:
                            return True
                    except ValueError:
                        pass
        time.sleep(1.0)
    return False


def stop(process, stream):
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=25.0)
        except KeyboardInterrupt:
            result["outcome"] = "interrupted_by_user"
            samples = out / "samples.json"
            progress = out / "waypoint_progress.json"
            if samples.exists():
                result.update(json.loads(samples.read_text(encoding="utf-8")))
            if progress.exists():
                result["partial_route_progress"] = json.loads(
                    progress.read_text(encoding="utf-8"))
            result["observer_interrupted_by_user"] = True
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
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            os.killpg(process.pid, 0)
            time.sleep(0.1)
        except ProcessLookupError:
            return True
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return True
    return False


def trial(experiment, output):
    name = experiment["name"]
    goals = experiment["goals"]
    continuous = experiment["continuous"]
    first_goal = goals[0]
    out = output / name
    out.mkdir(parents=True, exist_ok=True)
    processes = []
    stack_stopped = True
    landed = False
    result = {"name": name, "goal_lidar_m": first_goal,
              "route_goals_lidar_m": goals, "continuous_route": continuous,
              "outcome": "not_started"}
    ready_file = out / "hover_ready.flag"
    progress_file = out / "waypoint_progress.json"
    try:
        sim, stream = start(SOURCE + f"ros2 launch {PX4_LAUNCH} gui:=false",
                            out / "px4_gazebo.log", ROOT)
        processes.append((sim, stream))
        if not wait_topic("/clock", 120):
            result["outcome"] = "failed_no_clock"
            return result

        pgo, stream = start(
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/thunder/GRA_1/ws_livox/install/setup.bash && "
            "source /home/thunder/GRA_1/fastlio2_pgo_ws/install/setup.bash && "
            "ros2 launch pgo pgo_launch.py use_sim_time:=true",
            out / "fastlio_pgo.log", ROOT / "fastlio2_pgo_ws")
        processes.append((pgo, stream))
        if not wait_topic("/fastlio2/lio_odom", 90):
            result["outcome"] = "failed_no_fastlio_odom"
            return result

        launch = (
            f"ros2 launch px4ctrl bringup.launch.py start_agent:=true start_rviz:=false "
            f"start_takeoff:=false px4ctrl_params:={PX4_PARAMS} "
            "ego_max_vel:=1.0 ego_max_acc:=1.0 ego_max_jerk:=3.0 "
            "ego_lambda_smooth:=2.0 ego_planning_horizon:=7.5 "
            "ego_obstacle_inflation:=0.45 ego_collision_clearance:=0.65")
        bringup, stream = start(SOURCE + launch, out / "bringup.log", WS)
        processes.append((bringup, stream))
        if not wait_topic("/px4ctrl/status", 90) or not has_goal_subscriber(90):
            result["outcome"] = "failed_bringup_or_goal_subscription"
            return result

        observer_cmd = (SOURCE + f"python3 {OBSERVER} --csv {out/'samples.csv'} "
            f"--ready-file {ready_file} --goal-x {first_goal[0]} "
            f"--goal-y {first_goal[1]} --goal-z {first_goal[2]} "
            f"--route-json '{json.dumps(goals, separators=(',', ':'))}' "
            f"--timeout {420 if continuous else 120} --startup-timeout 300")
        observer, stream = start(observer_cmd, out / "observer.log", WS)
        processes.append((observer, stream))
        time.sleep(1.0)

        takeoff_rc = run_command(SOURCE +
            "ros2 run px4ctrl takeoff --altitude 0.5 --wait-timeout 45 --require-ev-fusion",
            out / "takeoff.log", WS, 90)
        result["takeoff_exit_code"] = takeoff_rc
        if takeoff_rc != 0:
            result["outcome"] = "failed_takeoff"
            return result

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not ready_file.exists():
            if observer.poll() is not None:
                break
            time.sleep(0.1)
        if not ready_file.exists():
            result["outcome"] = "failed_hover_baseline"
            return result
        result["hover_confirmed_before_goal"] = True
        observer_log = out / "observer.log"
        goal_exit_codes = []
        retry_exit_codes = []
        progress_deadline_sec = 300.0 if continuous else 180.0
        for index, goal in enumerate(goals):
            x, y, z = goal
            goal_command = (SOURCE +
                f"ros2 run px4ctrl manual_goal {x} {y} {z} --auto-yaw "
                "--wait-timeout 30 --require-subscriber-node drone_0_ego_planner_node")
            goal_log = out / ("manual_goal.log" if index == 0 else
                              f"manual_goal_waypoint_{index+1}.log")
            goal_exit_codes.append(run_command(goal_command, goal_log, WS, 60))
            marker = ("NED tracking window started from hover" if index == 0 else
                      f"Matching planner goal observed: waypoint {index+1}")
            for attempt in range(3):
                marker_deadline = time.monotonic() + 8.0
                while time.monotonic() < marker_deadline:
                    if marker in observer_log.read_text(encoding="utf-8", errors="replace"):
                        break
                    if observer.poll() is not None:
                        break
                    time.sleep(0.1)
                if marker in observer_log.read_text(encoding="utf-8", errors="replace"):
                    break
                if attempt < 2:
                    retry_exit_codes.append(run_command(
                        goal_command, out / f"manual_goal_waypoint_{index+1}_retry_{attempt+1}.log",
                        WS, 60))
            if marker not in observer_log.read_text(encoding="utf-8", errors="replace"):
                result["outcome"] = f"waypoint_{index+1}_goal_not_observed"
                break

            progress_deadline = time.monotonic() + progress_deadline_sec
            reached = False
            while time.monotonic() < progress_deadline:
                if progress_file.exists():
                    try:
                        progress = json.loads(progress_file.read_text(encoding="utf-8"))
                        reached = len(progress.get("completed_waypoints", [])) >= index + 1
                    except (json.JSONDecodeError, OSError):
                        reached = False
                if reached or observer.poll() is not None:
                    break
                time.sleep(0.2)
            if not reached:
                result["outcome"] = f"waypoint_{index+1}_not_reached_before_timeout"
                break

        result["goal_command_exit_codes"] = goal_exit_codes
        result["goal_retry_exit_codes"] = retry_exit_codes
        result["goal_retry_count"] = len(retry_exit_codes)
        try:
            result["observer_exit_code"] = observer.wait(timeout=30)
        except subprocess.TimeoutExpired:
            result["outcome"] = "observer_did_not_finish_after_final_waypoint"
        summary = out / "samples.json"
        if summary.exists():
            result.update(json.loads(summary.read_text(encoding="utf-8")))
        elif result.get("outcome") == "not_started":
            result["outcome"] = "missing_observer_summary"

        result["landing_exit_code"] = run_command(
            SOURCE + f"python3 {LANDER} --timeout 90", out / "landing.log", WS, 110)
        result["landing_confirmed"] = result["landing_exit_code"] == 0
        landed = True
        if result.get("trial_valid") and result["landing_confirmed"]:
            result["outcome"] = "arrived_all_waypoints" if continuous else "arrived"
        elif result.get("trial_valid"):
            result["outcome"] = "arrived_landing_unconfirmed"
        elif result.get("outcome") == "not_started":
            result["outcome"] = result.get("result", "observer_invalid")
    except KeyboardInterrupt:
        result["outcome"] = "interrupted_by_user"
        samples = out / "samples.json"
        progress = out / "waypoint_progress.json"
        if samples.exists():
            result.update(json.loads(samples.read_text(encoding="utf-8")))
        if progress.exists():
            result["partial_route_progress"] = json.loads(
                progress.read_text(encoding="utf-8"))
        result["observer_interrupted_by_user"] = True
    except subprocess.TimeoutExpired:
        result["outcome"] = "command_timeout"
    except Exception as exc:
        result["outcome"] = "exception"
        result["error"] = repr(exc)
    finally:
        if processes and not landed:
            try:
                result["landing_exit_code"] = run_command(
                    SOURCE + f"python3 {LANDER} --timeout 90",
                    out / "landing_fallback.log", WS, 110)
                result["landing_confirmed"] = result["landing_exit_code"] == 0
            except Exception as exc:
                result["landing_confirmed"] = False
                result["landing_error"] = repr(exc)
        for process, stream in reversed(processes):
            stack_stopped = stop(process, stream) and stack_stopped
        result["stack_shutdown_confirmed"] = stack_stopped
        (out / "result.json").write_text(
            json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--cases", help="Comma-separated experiment names")
    parser.add_argument("--ros-domain-id", type=int, default=81,
                        help="Dedicated DDS domain for the SITL stack (default: 81)")
    args = parser.parse_args()
    if not 1 <= args.ros_domain_id <= 101:
        parser.error("--ros-domain-id must be in [1, 101]")
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        parser.error(f"output directory must be new or empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    selected = EXPERIMENTS
    if args.cases:
        wanted = set(args.cases.split(","))
        unknown = wanted - {case["name"] for case in EXPERIMENTS}
        if unknown:
            parser.error("unknown case names: " + ", ".join(sorted(unknown)))
        selected = [case for case in EXPERIMENTS if case["name"] in wanted]
    results = []
    for i, case in enumerate(selected):
        item = trial(case, output)
        results.append(item)
        print(json.dumps({k: item.get(k) for k in (
                "name", "outcome", "result", "arrival_time_s",
            "px4_ned_tracking_error_rmse_m", "px4_ned_tracking_error_p95_m",
            "px4_ned_tracking_error_max_m", "px4_ned_final_tracking_error_m",
            "px4_ned_goal_error_final_m",
            "goal_retry_exit_codes", "landing_confirmed",
            "stack_shutdown_confirmed")}, allow_nan=False), flush=True)
        if not item.get("stack_shutdown_confirmed", False):
            results.extend({"name": c["name"], "route_goals_lidar_m": c["goals"],
                "outcome": "not_run_previous_stack_shutdown_unconfirmed",
                "landing_confirmed": False} for c in selected[i+1:])
            break
        if item.get("outcome") == "interrupted_by_user":
            results.extend({"name": c["name"], "route_goals_lidar_m": c["goals"],
                "outcome": "not_run_after_user_interruption",
                "landing_confirmed": False,
                "stack_shutdown_confirmed": False} for c in selected[i+1:])
            break
    summary = {"measurement_frame": "PX4 local NED only", "cases": results,
        "ros_domain_id": args.ros_domain_id,
        "complete": len(results) == len(selected) and all(
            x.get("outcome") == ("arrived_all_waypoints" if x.get("continuous_route") else "arrived")
            and x.get("landing_confirmed") and x.get("stack_shutdown_confirmed")
            for x in results),
        "selected_case_names": [c["name"] for c in selected]}
    (output / "suite_results.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False)+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
