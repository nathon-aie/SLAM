#!/usr/bin/env python3
"""RoboMaster EP sensor and motion tools for the SLAM project.

Examples:
    python main.py  # Interactive operation menu
    python main.py step-test --cells 1 --mock
    python main.py simulate --commands 'fwd 1, right' -y
    python main.py monitor --conn-type ap
    python main.py calibrate fit data/calibration_measurements.csv

    python main.py explore --mock
    python main.py explore --conn-type ap
"""

import argparse
import os
import signal
import sys
import time
from pathlib import Path

# Add src to sys.path
_SRC_DIR = Path(__file__).resolve().parent / "src"
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))


import json
from typing import List

from src.settings import get as setting, project_path
from src.robot_system import RobotSystem
from src.telemetry import TelemetryAnalyzer


def parse_custom_commands(cmd_input: str) -> List[str]:
    """Parses arbitrary command string or JSON list into robot controller commands."""
    if not cmd_input:
        return []
    s = cmd_input.strip()
    if s.startswith("["):
        try:
            return json.loads(s)
        except Exception:
            pass

    raw_items = [c.strip() for c in s.replace(";", ",").split(",") if c.strip()]
    parsed = []
    for item in raw_items:
        low = item.lower()
        if any(w in low for w in ("fwd", "forward", "move", "cell")):
            nums = [int(tok) for tok in item.split() if tok.isdigit()]
            cells = nums[0] if nums else 1
            parsed.append(f"Move Forward: {cells} cells")
        elif "left" in low:
            parsed.append("Turn Left (90 deg)")
        elif "right" in low:
            parsed.append("Turn Right (90 deg)")
        elif "around" in low or "180" in low:
            parsed.append("Turn Around (180 deg)")
        else:
            parsed.append(item)
    return parsed


def confirm_action(prompt: str, auto_yes: bool = False) -> bool:
    if auto_yes:
        return True
    try:
        answer = input(f"{prompt} [y/N]: ").strip().lower()
        if answer not in ("y", "yes"):
            print("[main] Operation cancelled.")
            return False
        return True
    except (EOFError, KeyboardInterrupt):
        return False


def run_navigation(sys_runner: RobotSystem, args):
    if not confirm_action("Start motion test now?", auto_yes=getattr(args, "yes", False)):
        return None

    # Set up motion test commands
    sys_runner.setup_threads()
    if hasattr(args, "commands") and args.commands and sys_runner.thread_2_controller:
        custom_cmds = parse_custom_commands(args.commands)
        sys_runner.thread_2_controller.set_commands(custom_cmds)
    if sys_runner.thread_2_controller:
        sys_runner.thread_2_controller.base_speed = args.speed
        sys_runner.thread_2_controller.wall_pid.nominal_side_dist_mm = args.nominal_side

    # Start sensor and motion workers
    sys_runner.start()
    commands_completed = sys_runner.wait_for_completion(
        timeout=args.duration if args.duration > 0 else None
    )

    return commands_completed


def cmd_simulate(args):
    print("=" * 65)
    print("🤖 STARTING STEP 3 MULTI-THREADING SIMULATION (PID GRID NAVIGATION)")
    print("=" * 65)
    sys_runner = RobotSystem(
        calibration_file=args.calibration,
        sensor_rate_hz=args.rate,
        mock_mode=True,
    )
    sys_runner.connect_robot()

    def sig_handler(sig, frame):
        print("\nInterrupt received. Stopping...")
        sys_runner.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    try:
        run_navigation(sys_runner, args)
    finally:
        sys_runner.shutdown()


def cmd_run(args):
    print("=" * 65)
    print("🤖 STARTING STEP 3 LIVE MULTI-THREADING RUN (PID GRID CONTROL)")
    print("=" * 65)
    sys_runner = RobotSystem(
        calibration_file=args.calibration,
        sensor_rate_hz=args.rate,
        mock_mode=False,
        conn_type=args.conn_type,
    )
    connected = sys_runner.connect_robot()
    if not connected and not args.allow_mock_fallback:
        print("[Error] Failed to connect to robot hardware. Exiting.")
        return 1

    def sig_handler(sig, frame):
        print("\nInterrupt received. Emergency stop initiated...")
        sys_runner.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    try:
        run_navigation(sys_runner, args)
    finally:
        sys_runner.shutdown()
    return 0


def cmd_step_test(args):
    print("=" * 65)
    print(f"🎯 TESTING {args.cells} GRID CELL(S) WITH STEP 3 PID CENTERING")
    print("=" * 65)
    sys_runner = RobotSystem(
        calibration_file=args.calibration,
        sensor_rate_hz=args.rate,
        mock_mode=args.mock,
        conn_type=args.conn_type,
    )
    sys_runner.connect_robot()
    sys_runner.setup_threads()

    if sys_runner.thread_2_controller:
        sys_runner.thread_2_controller.base_speed = args.speed
        sys_runner.thread_2_controller.wall_pid.nominal_side_dist_mm = args.nominal_side
        sys_runner.thread_2_controller.set_commands([f"Move Forward: {args.cells} cells"])

    def sig_handler(sig, frame):
        print("\nInterrupt received. Stopping...")
        sys_runner.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    sys_runner.start()
    sys_runner.wait_for_completion(timeout=setting("system.step_test_timeout_sec"))
    sys_runner.shutdown()


def cmd_turn_test(args):
    direction_str = str(args.direction).lower()
    if direction_str == "left":
        deg = 90.0
        cmd_text = "Turn Left (90 deg)"
    elif direction_str == "right":
        deg = -90.0
        cmd_text = "Turn Right (90 deg)"
    elif direction_str == "around":
        deg = 180.0
        cmd_text = "Turn Around (180 deg)"
    else:
        try:
            deg = float(direction_str)
            cmd_text = f"Turn {deg:+.0f} deg"
        except ValueError:
            print(f"Unknown direction '{args.direction}'. Use 'left', 'right', 'around', or degrees like '90' or '-90'.")
            return 1

    print("=" * 65)
    print(f"🔄 TESTING TURN: {cmd_text} (z={deg:+.0f}°)")
    print("=" * 65)

    sys_runner = RobotSystem(
        calibration_file=args.calibration,
        sensor_rate_hz=args.rate,
        mock_mode=args.mock,
        conn_type=args.conn_type,
    )
    sys_runner.connect_robot()
    sys_runner.setup_threads()

    if sys_runner.thread_2_controller:
        sys_runner.thread_2_controller.set_commands([cmd_text])

    def sig_handler(sig, frame):
        print("\nInterrupt received. Stopping...")
        sys_runner.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    sys_runner.start()
    sys_runner.wait_for_completion(timeout=setting("system.turn_test_timeout_sec"))
    sys_runner.shutdown()
    return 0


def cmd_monitor(args):
    print("=" * 65)
    print("📡 LIVE SENSOR MONITOR (THREAD 1)")
    print("=" * 65)
    sys_runner = RobotSystem(
        calibration_file=args.calibration,
        sensor_rate_hz=args.rate,
        mock_mode=args.mock,
        conn_type=args.conn_type,
    )
    sys_runner.connect_robot()
    sys_runner.setup_threads()

    def sig_handler(sig, frame):
        sys_runner.shutdown(save_telemetry=False)
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    sys_runner.thread_1_sensor.start_collecting()

    try:
        print(f"{'Frame':<8} | {'Sharp L (mm)':<13} | {'Sharp R (mm)':<13} | {'ToF (mm)':<10} | {'Yaw (deg)':<10} | {'Walls (L/F/R)'}")
        print("-" * 75)
        while True:
            state = sys_runner.sensor_hub.wait_for_next_state(timeout=1.0)
            if state:
                sl = f"{state.sharp_left_mm:.1f}" if state.sharp_left_mm is not None else "N/A"
                sr = f"{state.sharp_right_mm:.1f}" if state.sharp_right_mm is not None else "N/A"
                tof = f"{state.tof_filtered_mm:.1f}" if state.tof_filtered_mm is not None else "N/A"
                walls = f"{'L' if state.wall_left_detected else '-'}/{'F' if state.wall_front_detected else '-'}/{'R' if state.wall_right_detected else '-'}"
                print(f"{state.frame_index:<8} | {sl:<13} | {sr:<13} | {tof:<10} | {state.yaw:<10.1f} | {walls}")
            time.sleep(0.1)
    finally:
        sys_runner.shutdown(save_telemetry=False)


def cmd_analyze(args):
    print(f"Analyzing telemetry log: {args.file}")
    TelemetryAnalyzer.analyze_file(args.file, save_plot=not args.no_plot)


def run_calibration_session(args):
    from src.calibrate import CalibrationSession, fit_command
    from src.operation_menu import calibration_action
    session = CalibrationSession(getattr(args, "conn_type", setting("robot.conn_type")))
    action = args.sensor if args.cal_cmd == "collect-live" else "fit"
    status = 0
    try:
        while action is not None:
            try:
                if action == "fit":
                    fit_command(getattr(args, "input", project_path("paths.measurements")),
                                getattr(args, "output_dir", project_path("paths.calibration_output")))
                else:
                    # Explicit CLI port overrides apply to its original sensor only.
                    original_sensor = getattr(args, "sensor", None) == action
                    session.collect(action, getattr(args, "output", project_path("paths.measurements")),
                                    getattr(args, "board_id", None) if original_sensor else None,
                                    getattr(args, "port", None) if original_sensor else None,
                                    getattr(args, "tof_index", setting("sensors.tof_index")),
                                    getattr(args, "samples", setting("calibration.samples")))
            except (OSError, RuntimeError, ValueError) as exc:
                print("[calibration] {}".format(exc), file=sys.stderr)
                status = 1
            action = calibration_action()
    except (EOFError, KeyboardInterrupt):
        print("\n[calibration] ออกจากเมนู Calibration")
    finally:
        session.close()
    return status


def cmd_calibrate(args):
    if args.cal_cmd == "collect-live" or (args.cal_cmd == "fit" and getattr(args, "interactive_menu", False)):
        return run_calibration_session(args)
    try:
        from src.calibrate import collect_live, fit_command, init_csv
    except ImportError:
        from calibrate import collect_live, fit_command, init_csv
    try:
        if args.cal_cmd == "init-csv":
            init_csv(args.path)
        elif args.cal_cmd == "collect-live":
            collect_live(args.sensor, args.output, args.board_id, args.port, args.tof_index, args.samples, args.conn_type)
        elif args.cal_cmd == "fit":
            fit_command(args.input, args.output_dir)
    except (OSError, RuntimeError, ValueError) as exc:
        print("[calibration] {}".format(exc), file=sys.stderr)
        return 1
    except (EOFError, KeyboardInterrupt):
        print("\n[calibration] ยกเลิกการเก็บข้อมูล")
        return 0
    return 0


def cmd_explore(args):
    from src.grid_slam import DFSExplorer
    if args.mock:
        from src.slam_simulation import SimulationBackend
        explorer = DFSExplorer(SimulationBackend(), args.output)
        success = explorer.run()
    else:
        from src.slam_hardware import HardwareBackend
        system = RobotSystem(calibration_file=args.calibration, conn_type=args.conn_type)
        explorer = None
        try:
            if not system.connect_robot():
                return 1  # Exploration never silently falls back to a simulated world.
            system.setup_threads()
            system.thread_1_sensor.start_collecting()
            system.thread_2_controller._running.set()
            explorer = DFSExplorer(HardwareBackend(system), args.output)
            deadline = time.monotonic() + setting("slam.sensor_timeout_sec")
            while system.sensor_hub.get_latest_state().frame_index == 0:
                if time.monotonic() > deadline:
                    raise RuntimeError("No initial position/attitude data")
                time.sleep(0.01)
            success = explorer.run()
        except (Exception, KeyboardInterrupt) as exc:
            if explorer is None:
                print("[SLAM] Startup failed: {}".format(exc))
                return 1
            explorer.status = "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
            explorer.error = str(exc)
            explorer.slam.export(explorer.output, explorer.status, explorer.error)
            success = False
        finally:
            system.shutdown()
    from src.slam_report import save_run_report
    if args.mock:
        from src.telemetry import TelemetryRecorder
        recorder = TelemetryRecorder()
    else:
        recorder = system.telemetry
    archive, plot, log = save_run_report(explorer.output, recorder.run_dir,
        '{}_{}'.format(recorder.run_name, recorder.timestamp_str))
    print('[SLAM] archived map={}'.format(archive))
    print("[SLAM] map={} | actions={} | log={}".format(plot, plot.parent / "actions.html", log))
    print("[SLAM] {} | visited={} | moves={} | map={}".format(
        explorer.status, len(explorer.slam.map.visited), explorer.moves, explorer.output))
    if explorer.error:
        print("[SLAM] " + explorer.error)
    return 0 if success else 1


def cmd_evaluate_map(args):
    from src.slam_report import evaluate
    metrics = evaluate(args.map_file, args.ground_truth)
    output = Path(args.output) if args.output else Path(args.map_file).with_name(Path(args.map_file).stem + "_metrics.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metrics, indent=2))
    print("[SLAM] metrics={}".format(output))
    return 0


def main():
    parser = argparse.ArgumentParser(description="RoboMaster EP Grid SLAM and DFS Exploration")
    subparsers = parser.add_subparsers(dest="command", required=True)

    explore_p = subparsers.add_parser("explore", help="Build a map while exploring with Grid SLAM and DFS")
    explore_p.add_argument("--mock", action="store_true", help="Procedural sensor-world simulation")
    explore_p.add_argument("--output", default=project_path("slam.output"), help="Output map JSON (including partial runs)")
    explore_p.add_argument("--conn-type", choices=("ap", "sta"), default=setting("robot.conn_type"))
    explore_p.add_argument("--calibration", default=project_path("paths.calibration"))

    eval_p = subparsers.add_parser("evaluate-map", help="Compare an explored map with offline ground truth")
    eval_p.add_argument("map_file", help="Exploration output JSON")
    eval_p.add_argument("ground_truth", help="Post-run ground-truth JSON; never used for navigation")
    eval_p.add_argument("--output", help="Metrics JSON output")

    # 1. Run live
    run_p = subparsers.add_parser("run", help="Run explicit motion test commands on the robot")
    run_p.add_argument("--commands", required=True, help="Motion test commands (e.g. 'fwd 1, left, fwd 1')")
    run_p.add_argument("--calibration", default=project_path("paths.calibration"))
    run_p.add_argument("--conn-type", choices=("ap", "sta"), default=setting("robot.conn_type"))
    run_p.add_argument("--rate", type=float, default=setting("sensors.rate_hz"), help="Sensor collection rate Hz")
    run_p.add_argument("--speed", type=float, default=setting("navigation.base_speed_mps"), help="Base cruising speed (m/s)")
    run_p.add_argument("--nominal-side", type=float, default=setting("navigation.nominal_side_mm"), help="Nominal distance to single wall (mm)")
    run_p.add_argument("--duration", type=float, default=setting("navigation.duration_sec"), help="Max duration in seconds")
    run_p.add_argument("-y", "--yes", action="store_true", help="Auto-confirm all interactive prompts")
    run_p.add_argument("--allow-mock-fallback", action="store_true", help="Fallback to mock if robot unavailable")

    # 2. Simulate
    sim_p = subparsers.add_parser("simulate", help="Simulate explicit motion test commands")
    sim_p.add_argument("--commands", required=True, help="Motion test commands (e.g. 'fwd 1, left, fwd 1')")
    sim_p.add_argument("--calibration", default=project_path("paths.calibration"))
    sim_p.add_argument("--rate", type=float, default=setting("sensors.rate_hz"), help="Sensor collection rate Hz")
    sim_p.add_argument("--speed", type=float, default=setting("navigation.base_speed_mps"), help="Base cruising speed (m/s)")
    sim_p.add_argument("--nominal-side", type=float, default=setting("navigation.nominal_side_mm"), help="Nominal distance to single wall (mm)")
    sim_p.add_argument("--duration", type=float, default=setting("navigation.duration_sec"), help="Max duration in seconds")
    sim_p.add_argument("-y", "--yes", action="store_true", help="Auto-confirm all interactive prompts")

    # 3. Step-test
    step_p = subparsers.add_parser("step-test", help="Test N grid cell move with PID centering")
    step_p.add_argument("--cells", type=int, default=1, help="Number of cells to move")
    step_p.add_argument("--conn-type", choices=("ap", "sta"), default=setting("robot.conn_type"))
    step_p.add_argument("--calibration", default=project_path("paths.calibration"))
    step_p.add_argument("--rate", type=float, default=setting("sensors.rate_hz"))
    step_p.add_argument("--speed", type=float, default=setting("navigation.step_test_speed_mps"))
    step_p.add_argument("--nominal-side", type=float, default=setting("navigation.nominal_side_mm"))
    step_p.add_argument("--mock", action="store_true")

    # 4. Turn-test
    turn_p = subparsers.add_parser("turn-test", help="Test in-place turn (+90 right, -90 left, 180 around)")
    turn_p.add_argument("--direction", choices=("left", "right", "around"), default="right", help="Turn direction: left (z=-90), right (z=+90), around (z=180)")
    turn_p.add_argument("--conn-type", choices=("ap", "sta"), default=setting("robot.conn_type"))
    turn_p.add_argument("--calibration", default=project_path("paths.calibration"))
    turn_p.add_argument("--rate", type=float, default=setting("sensors.rate_hz"))
    turn_p.add_argument("--mock", action="store_true")

    # 5. Monitor
    mon_p = subparsers.add_parser("monitor", help="Live stream sensor telemetry (Thread 1)")
    mon_p.add_argument("--conn-type", choices=("ap", "sta"), default=setting("robot.conn_type"))
    mon_p.add_argument("--calibration", default=project_path("paths.calibration"))
    mon_p.add_argument("--rate", type=float, default=setting("sensors.rate_hz"))
    mon_p.add_argument("--mock", action="store_true", help="Monitor mock data")

    # 6. Analyze
    ana_p = subparsers.add_parser("analyze", help="Analyze telemetry log and generate graphs")
    ana_p.add_argument("file", help="Path to telemetry JSON file or run folder (e.g. telemetry_logs/run1)")
    ana_p.add_argument("--no-plot", action="store_true", help="Skip plot generation")

    # 7. Calibrate
    cal_p = subparsers.add_parser("calibrate", help="Sensor calibration tools (Step 1)")
    cal_sub = cal_p.add_subparsers(dest="cal_cmd", required=True)
    c_init = cal_sub.add_parser("init-csv", help="create CSV template")
    c_init.add_argument("path", nargs="?", default=project_path("paths.measurements"))
    c_live = cal_sub.add_parser("collect-live", help="collect live sensor values")
    c_live.add_argument("sensor", choices=("sharp_left", "sharp_right", "tof"))
    c_live.add_argument("--output", default=project_path("paths.measurements"))
    c_live.add_argument("--board-id", type=int)
    c_live.add_argument("--port", type=int)
    c_live.add_argument("--tof-index", type=int, default=setting("sensors.tof_index"))
    c_live.add_argument("--samples", type=int, default=setting("calibration.samples"))
    c_live.add_argument("--conn-type", choices=("ap", "sta"), default=setting("robot.conn_type"))
    c_fit = cal_sub.add_parser("fit", help="fit polynomial curves")
    c_fit.add_argument("input", default=project_path("paths.measurements"))
    c_fit.add_argument("--output-dir", default=project_path("paths.calibration_output"))


    cli_arguments = sys.argv[1:]
    interactive_menu = not cli_arguments
    if interactive_menu:
        from src.operation_menu import select_operation
        cli_arguments = select_operation()
        if cli_arguments is None:
            return 0
    args = parser.parse_args(cli_arguments)
    args.interactive_menu = interactive_menu
    if args.command in ("run", "simulate") and not parse_custom_commands(args.commands):
        parser.error("--commands must contain at least one motion test command")

    if args.command == "explore":
        return cmd_explore(args)
    elif args.command == "evaluate-map":
        return cmd_evaluate_map(args)
    elif args.command == "simulate":
        return cmd_simulate(args)
    elif args.command == "run":
        return cmd_run(args)
    elif args.command == "step-test":
        return cmd_step_test(args)
    elif args.command == "turn-test":
        return cmd_turn_test(args)
    elif args.command == "monitor":
        return cmd_monitor(args)
    elif args.command == "analyze":
        return cmd_analyze(args)
    elif args.command == "calibrate":
        return cmd_calibrate(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
