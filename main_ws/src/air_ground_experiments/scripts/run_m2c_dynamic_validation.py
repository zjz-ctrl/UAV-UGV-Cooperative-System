#!/usr/bin/env python3
"""Automated Dynamic M2-C A/B/C/D validation runner.

Runs each scenario in a fully isolated ROS/Gazebo/PX4 session (dedicated
ROS_MASTER_URI / GAZEBO_MASTER_URI ports), applies runtime observation-gate
scenario control, grades evidence with the pure m2c_validation module, and
writes per-scenario plus top-level summaries.

Usage:
    python3 src/air_ground_experiments/scripts/run_m2c_dynamic_validation.py \
        [--scenarios A B C D] [--dry-run] [--output-root DIR] [--seed N]
"""

import argparse
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time

WORKSPACE = str(
    Path(os.environ.get("UAV_UGV_ROOT", str(Path.home() / "UAV-UGV_ws"))) / "main_ws"
)
sys.path.insert(0, WORKSPACE + "/src/air_ground_experiments/src")
SETUP_SCRIPT = (
    WORKSPACE + "/src/air_ground_bringup/scripts/setup_mvp_env.sh"
)
LAUNCH_PACKAGE = "air_ground_bringup"
LAUNCH_FILE = "air_ground_inspection_experiment.launch"
CONTROL_TOPIC = "/air_ground_experiment/charuco/control"
DEFAULT_ROS_PORT = 11331
DEFAULT_GAZEBO_PORT = 11351

WIDE_A_BUDGET = (0.6, 0.12)
UAV_TF_FRAME = "air_ground_experiment/uav_odom"
UGV_TF_FRAME = "air_ground_experiment/ugv_odom"

COLLECT_TOPICS = (
    ("phase", "/air_ground/mission_phase"),
    ("action", "/air_ground/handoff/action"),
    ("confidence", "/air_ground/handoff/confidence_radius"),
    ("accepted", "/air_ground/registration/accepted_update"),
    ("revision", "/air_ground/registration/revision"),
    ("status", "/air_ground/registration/status"),
    ("valid", "/air_ground/registration/valid"),
    ("frozen", "/air_ground/registration/frozen"),
    ("inliers", "/air_ground/registration/inlier_count"),
    ("innovation", "/air_ground/registration/innovation"),
    ("estimate", "/air_ground/registration/estimate"),
    ("target_origin", "/air_ground/red_sphere/origin_point"),
    ("target_ugv", "/air_ground/red_sphere/ugv_odom_point"),
    ("goal", "/air_ground/ugv_goal"),
    ("obs_dest", "/air_ground_experiment/charuco/observation"),
)

SCENARIO_TIMEOUTS = {"A": 420.0, "B": 420.0, "C": 420.0, "D": 480.0}
RUNNER_MARKER_ARG = "run_m2c_dynamic_validation.py"


def reexec_with_environment(ros_port, gazebo_port):
    """Re-exec through the canonical MVP environment exactly once."""
    if os.environ.get("M2C_RUNNER_ENV") == "1":
        return
    package_path = os.environ.get("ROS_PACKAGE_PATH", "")
    if package_path.endswith("/opt/ros/noetic/share") and "PX4-Autopilot" in package_path:
        return
    command = (
        "source {setup} >/dev/null || exit 70\n"
        "export M2C_RUNNER_ENV=1\n"
        "export ROS_MASTER_URI=http://localhost:{ros_port}\n"
        "export GAZEBO_MASTER_URI=http://localhost:{gz_port}\n"
        'exec python3 -u "$0" "$@"\n'
    ).format(setup=SETUP_SCRIPT, ros_port=ros_port, gz_port=gazebo_port)
    os.execvp("bash", ["bash", "-c", command, os.path.abspath(__file__), *sys.argv[1:]])


def child_environment(ros_port, gazebo_port):
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://localhost:%d" % ros_port
    environment["GAZEBO_MASTER_URI"] = "http://localhost:%d" % gazebo_port
    environment["PYTHONUNBUFFERED"] = "1"
    return environment


def own_pid_chain():
    chain = set()
    pid = os.getpid()
    while pid > 0:
        chain.add(pid)
        try:
            status = Path("/proc/%d/status" % pid).read_text()
        except OSError:
            break
        for line in status.splitlines():
            if line.startswith("PPid:"):
                pid = int(line.split()[1])
                break
        else:
            break
    return chain


def processes_with_marker(marker):
    """Return (pid, cmdline) for processes whose environ carries our master URI."""
    matches = []
    excluded = own_pid_chain()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid in excluded:
            continue
        try:
            environ = (entry / "environ").read_bytes()
            cmdline = (entry / "cmdline").read_bytes().replace(b"\x00", b" ").decode(
                "utf-8", "replace"
            )
        except OSError:
            continue
        if marker.encode() in environ and RUNNER_MARKER_ARG not in cmdline:
            matches.append((pid, cmdline))
    return matches


def kill_leftover_processes(marker, logger):
    for round_index in range(4):
        matches = processes_with_marker(marker)
        if not matches:
            return True
        logger("leftover processes: %s" % [pid for pid, _ in matches])
        for pid, _ in matches:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
        time.sleep(3.0)
        matches = processes_with_marker(marker)
        for pid, _ in matches:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        time.sleep(2.0)
    return not processes_with_marker(marker)


def ports_free(ports):
    import socket

    for port in ports:
        probe = socket.socket()
        try:
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                return False
        finally:
            probe.close()
    return True


def kill_runner_instances():
    """Kill stale runner processes (exact script-name match, never self)."""
    victim_name = RUNNER_MARKER_ARG.encode()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            cmdline = (entry / "cmdline").read_bytes()
        except OSError:
            continue
        if victim_name in cmdline:
            try:
                os.kill(int(entry.name), signal.SIGKILL)
            except OSError:
                pass


def remove_stale_gazebo_state():
    """An unclean gzserver kill can leave ~/.gazebo state that stalls boot."""
    import shutil

    shutil.rmtree(Path.home() / ".gazebo", ignore_errors=True)


def ensure_clean_session(ros_port, gazebo_port, logger):
    marker = "ROS_MASTER_URI=http://localhost:%d" % ros_port
    kill_leftover_processes(marker, logger)
    remove_stale_gazebo_state()
    deadline = time.time() + 30.0
    while time.time() < deadline:
        if ports_free((ros_port, gazebo_port)) and not processes_with_marker(marker):
            return
        kill_leftover_processes(marker, logger)
        time.sleep(2.0)
    raise RuntimeError("session ports never became free")


def terminate_process_group(process, grace=20.0):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    except (OSError, ProcessLookupError):
        return
    deadline = time.time() + grace
    while time.time() < deadline:
        if process.poll() is not None:
            break
        time.sleep(0.5)
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except (OSError, ProcessLookupError):
            pass
        process.wait()


def wait_for_master(environment, timeout=90.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            result = subprocess.run(
                ["rosnode", "list"],
                env=environment,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=10,
            )
            if result.returncode == 0:
                return True
        except subprocess.SubprocessError:
            pass
        time.sleep(1.5)
    return False


def spawn_collectors(run_dir, environment):
    handles = []
    for name, topic in COLLECT_TOPICS:
        output = open(run_dir / ("%s.csv" % name), "w")
        errors = open(run_dir / ("%s.err" % name), "w")
        process = subprocess.Popen(
            ["rostopic", "echo", "-p", topic],
            stdout=output,
            stderr=errors,
            env=environment,
            start_new_session=True,
        )
        handles.append((name, process, output, errors))
    return handles


def stop_collectors(handles):
    for _, process, output, errors in handles:
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except (OSError, ProcessLookupError):
                process.terminate()
        output.close()
        errors.close()


def spawn_tf_snapshot(run_dir, environment, name, destination):
    output = open(run_dir / ("tf-%s.txt" % name), "w")
    process = subprocess.Popen(
        ["timeout", "8", "rosrun", "tf", "tf_echo", "air_ground_origin", destination],
        stdout=output,
        stderr=subprocess.STDOUT,
        env=environment,
        start_new_session=True,
    )
    return process, output


def send_control(environment, command, log_file):
    stamp = time.strftime("%H:%M:%S")
    try:
        result = subprocess.run(
            [
                "rostopic", "pub", "-1", CONTROL_TOPIC,
                "std_msgs/String", command,
            ],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=15,
            text=True,
        )
        line = "%s control=%s rc=%s\n" % (stamp, command, result.returncode)
    except subprocess.SubprocessError as error:
        line = "%s control=%s FAILED %s\n" % (stamp, command, error)
    log_file.write(line)
    log_file.flush()
    return "FAILED" not in line


class TailMonitor:
    """Incrementally read collector CSVs and expose quick scenario signals."""

    def __init__(self, run_dir):
        self.run_dir = run_dir
        self.lines = {name: [] for name, _ in COLLECT_TOPICS}
        self._offsets = {name: 0 for name, _ in COLLECT_TOPICS}
        self._partial = {name: "" for name, _ in COLLECT_TOPICS}

    def poll(self):
        for name, _ in COLLECT_TOPICS:
            path = self.run_dir / ("%s.csv" % name)
            try:
                with path.open("r") as stream:
                    stream.seek(self._offsets[name])
                    chunk = stream.read()
                    self._offsets[name] = stream.tell()
            except OSError:
                continue
            if not chunk:
                continue
            buffer = self._partial[name] + chunk
            complete, _, remainder = buffer.rpartition("\n")
            self._partial[name] = remainder
            if complete:
                self.lines[name].extend(complete.splitlines())

    def field_values(self, name, field_name):
        values = []
        for line in self.lines[name]:
            cells = line.split(",")
            if len(cells) < 2 or line.startswith("%"):
                continue
            try:
                values.append((float(cells[0]), cells[1]))
            except ValueError:
                continue
        return values

    def phases(self):
        return self.field_values("phase", "field.data")

    def actions(self):
        return self.field_values("action", "field.data")

    def revisions(self):
        numeric = []
        for stamp, value in self.field_values("revision", "field.data"):
            try:
                numeric.append((stamp, int(value)))
            except ValueError:
                continue
        return numeric

    def latest_revision(self):
        revisions = self.revisions()
        return revisions[-1][1] if revisions else 0


def scenario_launch_args(run_id, seed, budget):
    arguments = [
        "registration_mode:=opportunistic",
        "use_visual_frame_yaw:=true",
        "uncertainty_aware_handoff:=true",
        "inspection_radius:=%g" % budget[0],
        "inspection_yaw:=%g" % budget[1],
        "target_sigma_floor:=0.02",
        "reregistration_timeout:=60.0",
        "uav_x:=-3.0",
        "ugv_x:=-2.0",
        "registration_dx:=0.95",
        "registration_dy:=0.0",
        "registration_altitude:=1.8",
        "seed:=%d" % seed,
        "epoch_seconds:=0.0",
        "timeout_seconds:=180.0",
        "translational_drift_rate:=0.0",
        "yaw_drift_rate:=0.0",
        "drift_step_seconds:=1.0",
        "trial_id:=%s" % run_id,
        "output_directory:=/tmp/air_ground_experiments/%s" % run_id,
        "observation_control_topic:=%s" % CONTROL_TOPIC,
    ]
    return arguments


def load_events(run_dir):
    from air_ground_experiments.m2c_validation import (
        accepted_events,
        action_events,
        estimate_events,
        observation_times,
        parse_echo_csv,
        phase_events,
        scalar_events,
    )

    def rows(name):
        path = run_dir / ("%s.csv" % name)
        if not path.exists():
            return []
        return parse_echo_csv(path.read_text(errors="replace"))

    def scalar(name):
        return [
            (stamp, value)
            for stamp, value in scalar_events(rows(name))
        ]

    def text_scalar(name):
        return [
            (float(row["time"]), row.get("field.data", ""))
            for row in rows(name) if "time" in row
        ]

    def timed(name):
        return [float(row["time"]) for row in rows(name) if "time" in row]

    return dict(
        phases=phase_events(rows("phase")),
        actions=action_events(rows("action")),
        confidence=scalar("confidence"),
        inliers=[(stamp, int(value)) for stamp, value in scalar("inliers")],
        accepted=accepted_events(rows("accepted")),
        revision_values=scalar("revision"),
        estimates=estimate_events(rows("estimate")),
        statuses=text_scalar("status"),
        innovations=scalar("innovation"),
        goals=[(stamp, "goal") for stamp in timed("goal")],
        observation_dest_times=observation_times(rows("obs_dest")),
    )


def load_control_times(run_dir):
    path = run_dir / "control_events.csv"
    if not path.exists():
        return {}
    times = {}
    for line in path.read_text().splitlines()[1:]:
        cells = line.split(",")
        if len(cells) != 2:
            continue
        command, stamp = cells
        key = command.split()[0] if command.split() else command
        try:
            stamp_value = float(stamp)
        except ValueError:
            continue
        if abs(stamp_value) > 1.0e10:
            stamp_value /= 1.0e9
        times.setdefault(key, stamp_value)
    return times


def grade_scenario(scenario, run_dir, budget):
    import inspect

    from air_ground_experiments.m2c_validation import (
        grade_a,
        grade_b,
        grade_c,
        grade_d,
    )

    events = load_events(run_dir)
    controls = load_control_times(run_dir)
    if scenario == "A":
        grader, extra = grade_a, {
            "budget_radius": budget[0], "budget_yaw": budget[1]}
    elif scenario == "B":
        grader, extra = grade_b, {"hide_time": controls.get("hide")}
    elif scenario == "C":
        grader, extra = grade_c, {}
    else:
        grader, extra = grade_d, {
            "hide_time": controls.get("hide"),
            "outlier_time": controls.get("outlier"),
        }
    names = inspect.signature(grader).parameters
    merged = dict(events)
    merged.update(extra)
    return grader(**{key: value for key, value in merged.items() if key in names})


def write_run_summary(run_dir, label, verdict, extra=None):
    lines = ["SCENARIO %s: %s" % (label, verdict.status)]
    lines += ["REASON: %s" % reason for reason in verdict.reasons] or ["REASON: none"]
    lines.append("EVIDENCE:")
    for key in sorted(verdict.evidence):
        lines.append("  %s=%s" % (key, verdict.evidence[key]))
    if extra:
        lines.append("NOTE: %s" % extra)
    (run_dir / "summary.txt").write_text("\n".join(lines) + "\n")
    document = {
        "scenario": verdict.scenario,
        "status": verdict.status,
        "reasons": list(verdict.reasons),
        "evidence": {
            key: (
                value if isinstance(value, (int, str, type(None)))
                else float(value)
            )
            for key, value in verdict.evidence.items()
        },
    }
    if extra:
        document["note"] = extra
    (run_dir / "verdict.json").write_text(json.dumps(document, indent=2) + "\n")


def prepare_run_directory(run_dir):
    if run_dir.exists():
        shutil.rmtree(run_dir)
    run_dir.mkdir(parents=True)


class ScenarioDriver:
    def __init__(self, args, logger):
        self.ros_port = args.ros_port
        self.gazebo_port = args.gazebo_port
        self.seed = args.seed
        self.output_root = Path(args.output_root)
        self.logger = logger

    def environment(self):
        return child_environment(self.ros_port, self.gazebo_port)

    def run_single(self, label, run_id, scenario, budget, timeout):
        environment = self.environment()
        run_dir = self.output_root / label
        prepare_run_directory(run_dir)
        log_file = open(run_dir / "launch.log", "w")
        self.logger("[%s] launching (logs: %s)" % (label, run_dir))
        launch = subprocess.Popen(
            [
                "roslaunch", LAUNCH_PACKAGE, LAUNCH_FILE,
                *scenario_launch_args(run_id, self.seed, budget),
            ],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
        collectors = []
        tf_handles = []
        control_log = open(run_dir / "control.log", "w")
        control_events_path = run_dir / "control_events.csv"
        control_events_file = open(control_events_path, "w")
        control_events_file.write("command,sim_time\n")
        control_events_file.flush()

        def issue_control(command, sim_time):
            sent = send_control(environment, command, control_log)
            if sent:
                stamp_seconds = sim_time / 1.0e9 if abs(sim_time) > 1.0e10 else sim_time
                control_events_file.write("%s,%f\n" % (command, stamp_seconds))
                control_events_file.flush()
            return sent

        monitor = TailMonitor(run_dir)
        outcome = "TIMEOUT"
        try:
            if not wait_for_master(environment):
                outcome = "LAUNCH_FAILED"
                self.logger("[%s] ROS master never came up" % label)
                return outcome, run_dir
            collectors = spawn_collectors(run_dir, environment)
            sim_time = 0.0
            first_revision_time = None
            wait_entered_time = None
            hide_sent = False
            outlier_sent = False
            reregister_seen = False
            direct_seen = False
            goal_seen = False
            deadline = time.time() + timeout
            last_progress = time.time()
            ever_progress = False
            while time.time() < deadline:
                if launch.poll() is not None:
                    outcome = "LAUNCH_EXITED"
                    break
                monitor.poll()
                phase_values = monitor.phases()
                action_values = monitor.actions()
                revision_values = monitor.revisions()
                goal_rows = [
                    row for row in monitor.lines["goal"] if not row.startswith("%")
                ]
                if phase_values:
                    sim_time = max(sim_time, phase_values[-1][0])
                    ever_progress = True
                    last_progress = time.time()
                elif not ever_progress and time.time() - last_progress > 150.0:
                    outcome = "INFRA_STALLED"
                    self.logger(
                        "[%s] infrastructure stalled before mission start" % label)
                    break
                accepted_first = next(
                    ((stamp, value) for stamp, value in revision_values if value >= 1),
                    None,
                )

                if accepted_first is not None and first_revision_time is None:
                    first_revision_time = accepted_first[0]
                    time.sleep(2.0)
                    tf_handles.append(spawn_tf_snapshot(
                        run_dir, environment, "origin-uav", UAV_TF_FRAME))
                    tf_handles.append(spawn_tf_snapshot(
                        run_dir, environment, "origin-ugv", UGV_TF_FRAME))

                actions_seen = {value for _, value in action_values}
                phases_seen = {value for _, value in phase_values}
                direct_seen = direct_seen or "DIRECT" in actions_seen
                reregister_seen = reregister_seen or "REREGISTER" in actions_seen
                goal_seen = goal_seen or bool(goal_rows)

                if scenario == "B" or scenario == "D":
                    if (not hide_sent and first_revision_time is not None
                            and sim_time >= first_revision_time + 1.0):
                        hide_sent = issue_control("hide", sim_time)
                        self.logger("[%s] hide sent (sim %.1fs)" % (label, sim_time))
                if scenario == "D":
                    wait_now = next(
                        (stamp for stamp, value in phase_values
                         if value == "WAIT_REREGISTRATION"),
                        None,
                    )
                    if wait_now is not None and wait_entered_time is None:
                        wait_entered_time = wait_now
                    if (wait_entered_time is not None and not outlier_sent
                            and hide_sent and sim_time >= wait_entered_time + 4.0):
                        outlier_sent = issue_control("outlier 1.5 0.8 0.5", sim_time)
                        self.logger("[%s] outlier sent (sim %.1fs)" % (label, sim_time))

                if scenario == "A":
                    if direct_seen and "OVERWATCH" in phases_seen and goal_seen:
                        outcome = "COMPLETED"
                        time.sleep(6.0)
                        break
                    if reregister_seen and "RETURN_TO_UGV" in phases_seen:
                        outcome = "REREGISTER_DETECTED"
                        time.sleep(8.0)
                        break
                elif scenario == "B":
                    if "WAIT_REREGISTRATION" in phases_seen:
                        outcome = "COMPLETED"
                        time.sleep(12.0)
                        break
                elif scenario == "C":
                    revision_now = monitor.latest_revision()
                    if revision_now >= 2 and "OVERWATCH" in phases_seen and goal_seen:
                        outcome = "COMPLETED"
                        time.sleep(6.0)
                        break
                elif scenario == "D":
                    if outlier_sent:
                        outcome = "COMPLETED"
                        time.sleep(25.0)
                        break
                time.sleep(1.0)
            else:
                outcome = "TIMEOUT"
        finally:
            stop_collectors(collectors)
            for process, output in tf_handles:
                if process.poll() is None:
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    except (OSError, ProcessLookupError):
                        pass
                output.close()
            control_log.close()
            control_events_file.close()
            terminate_process_group(launch)
            log_file.close()
            kill_leftover_processes(
                "ROS_MASTER_URI=http://localhost:%d" % self.ros_port, self.logger)
            if not ports_free((self.ros_port, self.gazebo_port)):
                self.logger("[%s] WARNING: ports not free after cleanup" % label)
        return outcome, run_dir

    def run_with_retry(self, label, run_id, scenario, budget, timeout):
        outcome, run_dir = self.run_single(
            label, run_id, scenario, budget, timeout)
        if outcome in ("INFRA_STALLED", "LAUNCH_FAILED", "LAUNCH_EXITED"):
            self.logger(
                "[%s] infrastructure outcome %s; cleaning and retrying once"
                % (label, outcome))
            remove_stale_gazebo_state()
            outcome, run_dir = self.run_single(
                label + "-retry",
                run_id + "r",
                scenario,
                budget,
                timeout,
            )
        return run_dir

    def run_a(self):
        from air_ground_experiments.m2c_validation import (
            DEFAULT_INSPECTION_RADIUS,
            DEFAULT_INSPECTION_YAW,
        )

        stamp = time.strftime("%Y%m%d-%H%M%S")
        first_budget = (DEFAULT_INSPECTION_RADIUS, DEFAULT_INSPECTION_YAW)
        run_dir = self.run_with_retry(
            "A_direct/attempt1",
            "m2c-a1-%s" % stamp,
            "A",
            first_budget,
            SCENARIO_TIMEOUTS["A"],
        )
        verdict = grade_scenario("A", run_dir, first_budget)
        write_run_summary(run_dir, "A attempt1 (default budget)", verdict)
        if verdict.status == "PASS":
            return verdict, "default budget satisfied DIRECT"
        if verdict.status != "INCONCLUSIVE":
            return verdict, "default-budget attempt failed (see summary)"
        self.logger("[A] INCONCLUSIVE_A_INPUT at default budget; retrying with "
                    "scenario budget %s" % (WIDE_A_BUDGET,))
        run_dir = self.run_with_retry(
            "A_direct/attempt2",
            "m2c-a2-%s" % stamp,
            "A",
            WIDE_A_BUDGET,
            SCENARIO_TIMEOUTS["A"],
        )
        verdict2 = grade_scenario("A", run_dir, WIDE_A_BUDGET)
        status = verdict2.status
        extra = (
            "attempt1 at default budget was INCONCLUSIVE_A_INPUT "
            "(confidence=%s); attempt2 used documented scenario budget %s"
            % (verdict.evidence.get("confidence"), (WIDE_A_BUDGET,))
        )
        if status == "INCONCLUSIVE":
            status = "FAIL"
            extra += "; second attempt also inconclusive"
        verdict2.status = status
        write_run_summary(run_dir, "A attempt2 (scenario budget)", verdict2, extra)
        return verdict2, extra

    def run(self, scenarios):
        results = {}
        if "A" in scenarios:
            verdict, note = self.run_a()
            results["A"] = (verdict.status, verdict, note)
        plans = (
            ("B", "B_reregister", (0.35, 0.03490658503988659)),
            ("C", "C_resume", (0.35, 0.03490658503988659)),
            ("D", "D_outlier", (0.35, 0.03490658503988659)),
        )
        for scenario, label, budget in plans:
            if scenario not in scenarios:
                continue
            stamp = time.strftime("%Y%m%d-%H%M%S")
            run_dir = self.run_with_retry(
                label,
                "m2c-%s-%s" % (scenario.lower(), stamp),
                scenario,
                budget,
                SCENARIO_TIMEOUTS[scenario],
            )
            verdict = grade_scenario(scenario, run_dir, budget)
            write_run_summary(run_dir, label, verdict)
            results[scenario] = (verdict.status, verdict, "")
        return results


def dry_run(args):
    logger = make_logger()
    logger("dry-run: resolving canonical environment via setup script")
    environment = child_environment(args.ros_port, args.gazebo_port)
    checks = []
    for package in ("air_ground_bringup", "air_ground_experiments"):
        result = subprocess.run(
            ["rospack", "find", package],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=30,
        )
        checks.append((("rospack %s" % package), result.returncode == 0,
                       result.stdout.strip()))
    argv = [
        "roslaunch", LAUNCH_PACKAGE, LAUNCH_FILE,
        *scenario_launch_args("dry-run-probe", args.seed, (0.35, 0.03490658503988659)),
    ]
    checks.append(("launch argv built", len(argv) > 3, " ".join(argv)))
    checks.append(("ports free", ports_free((args.ros_port, args.gazebo_port)),
                   "%d/%d" % (args.ros_port, args.gazebo_port)))
    marker = "ROS_MASTER_URI=http://localhost:%d" % args.ros_port
    checks.append(("no leftover session", not processes_with_marker(marker), marker))
    ok = True
    for name, passed, detail in checks:
        logger("  %-28s %s %s" % (name, "OK" if passed else "FAIL", detail))
        ok = ok and passed
    logger("DRY_RUN_%s" % ("OK" if ok else "FAILED"))
    return 0 if ok else 1


def make_logger():
    def logger(message):
        print("[%s] %s" % (time.strftime("%H:%M:%S"), message), flush=True)
    return logger


def write_top_summary(output_root, results):
    labels = {
        "A": "A DIRECT           ",
        "B": "B REREGISTER       ",
        "C": "C REVISION2_RESUME ",
        "D": "D OUTLIER_REJECT   ",
    }
    evidence_keys = {
        "A": ("revision1", "inliers", "confidence", "sigma_xy", "sigma_yaw_deg",
              "DIRECT", "DISPATCH", "OVERWATCH", "GOAL"),
        "B": ("revision1", "max_revision", "cov_before", "cov_after",
              "REREGISTER", "RETURN_TO_UGV", "WAIT_REREGISTRATION", "DISPATCH",
              "GOAL"),
        "C": ("revision_before", "revision_after", "cov_before", "cov_after",
              "sigma_xy_before", "sigma_xy_after", "sigma_yaw_before",
              "sigma_yaw_after", "RESUME_HANDOFF", "DISPATCH", "GOAL"),
        "D": ("revision_before", "revision_after", "NIS", "REJECTED",
              "RESUME_HANDOFF", "DISPATCH", "GOAL", "cov_before",
              "cov_min_after"),
    }
    lines = ["M2-C DYNAMIC VALIDATION", ""]
    for scenario in ("A", "B", "C", "D"):
        if scenario not in results:
            continue
        status = results[scenario][0]
        lines.append("%s %s" % (labels[scenario], status))
    lines.append("")
    for scenario in ("A", "B", "C", "D"):
        if scenario not in results:
            continue
        verdict = results[scenario][1]
        note = results[scenario][2]
        lines.append("%s:" % scenario)
        for key in evidence_keys[scenario]:
            lines.append("  %s=%s" % (key, verdict.evidence.get(key)))
        for reason in verdict.reasons:
            lines.append("  reason: %s" % reason)
        if note:
            lines.append("  note: %s" % note)
        lines.append("")
    path = output_root / "summary.txt"
    path.write_text("\n".join(lines) + "\n")
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", nargs="+", default=["A", "B", "C", "D"],
                        choices=("A", "B", "C", "D"))
    parser.add_argument("--output-root", default="/tmp/air_ground_m2c_dynamic")
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--ros-port", type=int, default=DEFAULT_ROS_PORT)
    parser.add_argument("--gazebo-port", type=int, default=DEFAULT_GAZEBO_PORT)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    reexec_with_environment(args.ros_port, args.gazebo_port)
    logger = make_logger()
    if args.dry_run:
        return dry_run(args)

    kill_runner_instances()
    ensure_clean_session(args.ros_port, args.gazebo_port, logger)
    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    driver = ScenarioDriver(args, logger)
    results = driver.run(set(args.scenarios))
    path = write_top_summary(output_root, results)
    logger("summary written to %s" % path)
    failed = [s for s, (status, _, _) in results.items() if status != "PASS"]
    logger("RESULTS: %s" % ", ".join(
        "%s=%s" % (s, results[s][0]) for s in sorted(results)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
