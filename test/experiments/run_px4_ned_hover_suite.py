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
WS = ROOT / "px4ctrl_ws2"
DEFAULT_OUT = WS / "test/experiments/2026-11-13_px4_ned_hover_start_five_point"
OBSERVER = WS / "test/experiments/px4_ned_hover_observer.py"
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


def trial(case, output):
    name, x, y, z = case
    out = output / name
    out.mkdir(parents=True, exist_ok=True)
    processes = []
    stack_stopped = True
    landed = False
    result = {"name": name, "goal_lidar_m": [x, y, z], "outcome": "not_started"}
    ready_file = out / "hover_ready.flag"
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
            f"--ready-file {ready_file} --goal-x {x} --goal-y {y} --goal-z {z} "
            "--timeout 120 --startup-timeout 300")
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
        goal_command = (SOURCE +
            f"ros2 run px4ctrl manual_goal {x} {y} {z} --auto-yaw --wait-timeout 30")
        result["goal_command_exit_code"] = run_command(
            goal_command, out / "manual_goal.log", WS, 60)
        retry_exit_codes = []
        observer_log = out / "observer.log"
        for attempt in range(3):
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline:
                if "NED tracking window started from hover" in observer_log.read_text(
                        encoding="utf-8", errors="replace"):
                    break
                if observer.poll() is not None:
                    break
                time.sleep(0.1)
            if "NED tracking window started from hover" in observer_log.read_text(
                    encoding="utf-8", errors="replace"):
                break
            if attempt < 2:
                retry_exit_codes.append(run_command(
                    goal_command, out / f"manual_goal_retry_{attempt+1}.log", WS, 60))
        result["goal_retry_exit_codes"] = retry_exit_codes
        result["goal_retry_count"] = len(retry_exit_codes)
        try:
            result["observer_exit_code"] = observer.wait(timeout=200)
        except subprocess.TimeoutExpired:
            result["outcome"] = "observer_process_timeout"
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
            result["outcome"] = "arrived"
        elif result.get("trial_valid"):
            result["outcome"] = "arrived_landing_unconfirmed"
        elif result.get("outcome") == "not_started":
            result["outcome"] = result.get("result", "observer_invalid")
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
    parser.add_argument("--cases", help="Comma-separated case names")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        parser.error(f"output directory must be new or empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    selected = CASES
    if args.cases:
        wanted = set(args.cases.split(","))
        unknown = wanted - {case[0] for case in CASES}
        if unknown:
            parser.error("unknown case names: " + ", ".join(sorted(unknown)))
        selected = [case for case in CASES if case[0] in wanted]
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
            results.extend({"name": c[0], "goal_lidar_m": list(c[1:]),
                "outcome": "not_run_previous_stack_shutdown_unconfirmed",
                "landing_confirmed": False} for c in selected[i+1:])
            break
    summary = {"measurement_frame": "PX4 local NED only", "cases": results,
        "complete": len(results) == len(selected) and all(
            x.get("outcome") != "not_run_previous_stack_shutdown_unconfirmed" for x in results),
        "selected_case_names": [c[0] for c in selected]}
    (output / "suite_results.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False)+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
