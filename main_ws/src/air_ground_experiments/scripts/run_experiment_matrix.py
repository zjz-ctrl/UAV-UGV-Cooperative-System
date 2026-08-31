#!/usr/bin/env python3
"""Deterministic one-shot baseline matrix expansion and cold-start trials."""

import math
import signal

import numpy as np


TRIAL_ID_PREFIX = "one_shot"

EXIT_PASS = "PASS"
EXIT_LAUNCH_FAILED = "LAUNCH_FAILED"
EXIT_TIMEOUT = "TIMEOUT"
EXIT_REGISTRATION_FAILED = "REGISTRATION_FAILED"
EXIT_MISSION_FAILED = "MISSION_FAILED"

ROS_PROCESS_PATTERNS = (
    "roslaunch",
    "rosmaster",
    "roscore",
    "gzserver",
    "gzclient",
    "px4",
)


class ProcessPrecheckError(RuntimeError):
    """Raised when a matching ROS/Gazebo/PX4 process is still running."""


def matching_processes(patterns, cmdline_by_pid):
    """Return pids whose command line contains any forbidden pattern."""
    matched = []
    for pid, command_line in cmdline_by_pid.items():
        if any(pattern in command_line for pattern in patterns):
            matched.append(str(pid))
    return matched


def scan_running_processes():
    """Snapshot pid -> command line for every readable /proc entry."""
    from pathlib import Path

    running = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
        except OSError:
            continue
        running[entry.name] = raw.replace(b"\x00", b" ").decode(
            "utf-8", "replace"
        )
    return running


# Failure taxonomy: recorder failure codes mapped to runner exit codes.
REGISTRATION_FAILURE_CODES = frozenset({
    "MISSION_REGISTRATION",
    "INCOMPLETE_TRUTH_SYNC",
    "ANOMALY_TRUTH_UNAVAILABLE",
})


def classify_trial_row(row):
    """Map a canonical recorder row to (exit classification, raw code)."""
    status = str(row["status"]).upper().strip()
    failure_code = str(row.get("failure_code", "") or "")
    if status == "COMPLETED":
        return (EXIT_PASS, "") if row.get("success") else (
            EXIT_MISSION_FAILED, failure_code)
    if status == "TIMEOUT" or failure_code == "TRIAL_TIMEOUT":
        return EXIT_TIMEOUT, failure_code
    if failure_code in REGISTRATION_FAILURE_CODES:
        return EXIT_REGISTRATION_FAILED, failure_code
    return EXIT_MISSION_FAILED, failure_code


class JsonClassificationWriter:
    """Persist one exit-classification document per trial output directory."""

    def write(self, spec, document):
        import json
        from pathlib import Path

        directory = Path(spec.output_directory)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / "exit.json"
        with path.open("w") as stream:
            json.dump(document, stream, indent=2, sort_keys=True)
            stream.write("\n")
        return str(path)


def safe_trial_file_stem(trial_id):
    """Mirror TrialResultWriter's filesystem-safe trial id rule."""
    return "".join(
        character if character.isalnum() or character in "-_." else "_"
        for character in str(trial_id)
    )


def stale_trial_ids(specs):
    """Trial ids whose output directory already holds prior-run results."""
    from pathlib import Path

    stale = []
    for spec in specs:
        directory = Path(spec.output_directory)
        if not directory.is_dir():
            continue
        markers = (
            "{}.json".format(safe_trial_file_stem(spec.trial_id)),
            "exit.json",
        )
        if any((directory / marker).exists() for marker in markers):
            stale.append(spec.trial_id)
    return stale


def evaluate_stale_outputs(specs, force=False):
    """Preflight reruns: refuse stale results unless explicitly cleared.

    Returns (status, trial_ids): ``("ok", [])`` when every directory is
    fresh, ``("conflict", ids)`` when stale results exist and ``force`` is
    false, or ``("cleared", ids)`` after removing those entire directories
    so a rerun cannot mix rows from different matrix invocations.
    """
    import shutil

    stale = stale_trial_ids(specs)
    if not stale:
        return "ok", []
    if not force:
        return "conflict", stale
    for spec in specs:
        if spec.trial_id in set(stale):
            shutil.rmtree(spec.output_directory, ignore_errors=True)
    return "cleared", stale


def apply_output_root(specs, root):
    """Return copies of the specs redirected under an alternative root."""
    from pathlib import Path

    redirected = []
    for spec in specs:
        directory = str(Path(root) / spec.trial_id)
        redirected.append(
            TrialSpec(
                spec.trial_id,
                spec.seed,
                dict(spec.launch_args, output_directory=directory),
                spec.timeout_seconds,
                directory,
                spec.sampled_parameters,
            )
        )
    return redirected


class JsonResultReader:
    """Read the recorder's per-trial JSON document, if it exists yet."""

    def read(self, spec):
        import json
        from pathlib import Path

        path = (
            Path(spec.output_directory)
            / "{}.json".format(safe_trial_file_stem(spec.trial_id))
        )
        if not path.exists():
            return None
        with path.open() as stream:
            return json.load(stream)
SAMPLING_KEYS = (
    "uav_x",
    "uav_y",
    "uav_yaw_deg",
    "ugv_heading_offset_deg",
    "uav_frame_xy_m",
    "uav_frame_xy_m",
    "uav_frame_yaw_deg",
    "ugv_frame_xy_m",
    "ugv_frame_xy_m",
    "ugv_frame_yaw_deg",
)
REQUIRED_BOUNDS = (
    "uav_x",
    "uav_y",
    "uav_yaw_deg",
    "ugv_heading_offset_deg",
    "uav_frame_xy_m",
    "uav_frame_yaw_deg",
    "ugv_frame_xy_m",
    "ugv_frame_yaw_deg",
)


class TrialSpec:
    """One immutable cold-start trial of the one-shot registration matrix."""

    def __init__(self, trial_id, seed, launch_args, timeout_seconds,
                 output_directory, sampled_parameters=None):
        self.trial_id = str(trial_id)
        self.seed = int(seed)
        self.launch_args = dict(launch_args)
        self.timeout_seconds = float(timeout_seconds)
        self.output_directory = str(output_directory)
        self.sampled_parameters = dict(sampled_parameters or {})

    def _comparison_key(self):
        return (
            self.trial_id,
            self.seed,
            tuple(sorted(self.launch_args.items())),
            self.timeout_seconds,
            self.output_directory,
            tuple(sorted(self.sampled_parameters.items())),
        )

    def __eq__(self, other):
        return isinstance(other, TrialSpec) and (
            self._comparison_key() == other._comparison_key()
        )

    def __repr__(self):
        return "TrialSpec({!r}, {!r})".format(self.trial_id, self.seed)


def _normalize_angle_deg(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


def _sample_trial_parameters(seed, bounds):
    """Draw every pose from an instance-local generator seeded by the trial seed."""
    rng = np.random.default_rng(seed)
    return {
        key: float(rng.uniform(float(low), float(high)))
        for key, low, high in (
            (key, *bounds[key]) for key in SAMPLING_KEYS
        )
    }


def _trial_launch_args(trial_id, seed, sampled, config, output_directory):
    dx = float(config["registration_dx_m"])
    dy = float(config["registration_dy_m"])
    uav_yaw_rad = math.radians(sampled["uav_yaw_deg"])
    ugv_x = sampled["uav_x"] + math.cos(uav_yaw_rad) * dx - math.sin(uav_yaw_rad) * dy
    ugv_y = sampled["uav_y"] + math.sin(uav_yaw_rad) * dx + math.cos(uav_yaw_rad) * dy
    ugv_yaw = _normalize_angle_deg(
        sampled["uav_yaw_deg"] + sampled["ugv_heading_offset_deg"]
    )
    return {
        "registration_mode": str(config.get("mode", "one_shot")),
        "use_visual_frame_yaw": "true",
        "seed": str(int(seed)),
        "trial_id": trial_id,
        "output_directory": str(output_directory),
        "uav_x": repr(sampled["uav_x"]),
        "uav_y": repr(sampled["uav_y"]),
        "uav_yaw": repr(sampled["uav_yaw_deg"]),
        "ugv_x": repr(ugv_x),
        "ugv_y": repr(ugv_y),
        "ugv_yaw": repr(ugv_yaw),
        "registration_dx": repr(dx),
        "registration_dy": repr(dy),
        # Pin the acquisition altitude explicitly so frozen baseline trials
        # never drift when the launch default moves for manual runs.
        "registration_altitude": repr(
            float(config.get("registration_altitude_m", 1.5))
        ),
        "uav_initial_xyyaw": "[{}, {}, {}]".format(
            repr(sampled["uav_frame_xy_m"]),
            repr(sampled["uav_frame_xy_m"]),
            repr(math.radians(sampled["uav_frame_yaw_deg"])),
        ),
        "ugv_initial_xyyaw": "[{}, {}, {}]".format(
            repr(sampled["ugv_frame_xy_m"]),
            repr(sampled["ugv_frame_xy_m"]),
            repr(math.radians(sampled["ugv_frame_yaw_deg"])),
        ),
        "translational_drift_rate": "0.0",
        "yaw_drift_rate": "0.0",
        "drift_step_seconds": "1.0",
        "epoch_seconds": repr(float(config.get("epoch_seconds", 0.0))),
        "timeout_seconds": repr(float(config["timeout_seconds"])),
    }


def expand_matrix(config):
    """Expand the frozen matrix config into per-seed TrialSpec values."""
    if config.get("drift", "zero") != "zero":
        raise ValueError("the one-shot baseline matrix requires drift: zero")
    missing = [name for name in REQUIRED_BOUNDS if name not in config["bounds"]]
    if missing:
        raise ValueError("missing sampling bounds: {}".format(missing))
    trials = []
    for rank, seed in enumerate(sorted(int(value) for value in config["seeds"])):
        trial_id = "{}-{:04d}".format(TRIAL_ID_PREFIX, rank)
        sampled = _sample_trial_parameters(seed, config["bounds"])
        output_directory = "{}/{}".format(
            str(config["output_root"]).rstrip("/"), trial_id
        )
        trials.append(
            TrialSpec(
                trial_id,
                seed,
                _trial_launch_args(
                    trial_id, seed, sampled, config, output_directory
                ),
                float(config["timeout_seconds"]),
                output_directory,
                sampled,
            )
        )
    return sorted(trials, key=lambda trial: trial.trial_id)


def roslaunch_command(spec):
    """Return the explicit cold-start roslaunch argv for one trial."""
    argv = [
        "roslaunch",
        "air_ground_bringup",
        "air_ground_inspection_experiment.launch",
    ]
    for name, value in spec.launch_args.items():
        argv.append("{}:={}".format(name, value))
    return argv


class TrialRunner:
    """Sequential cold-start lifecycle driven entirely by injected collaborators."""

    def __init__(
        self,
        process_manager,
        clock,
        frozen_watch_factory,
        result_reader,
        classification_writer,
        flush_recorder=None,
        registration_timeout_seconds=120.0,
        poll_interval_seconds=1.0,
        shutdown_grace_seconds=15.0,
    ):
        self.process_manager = process_manager
        self.clock = clock
        self.frozen_watch_factory = frozen_watch_factory
        self.result_reader = result_reader
        self.classification_writer = classification_writer
        self.flush_recorder = flush_recorder
        self.registration_timeout_seconds = float(registration_timeout_seconds)
        self.poll_interval_seconds = float(poll_interval_seconds)
        self.shutdown_grace_seconds = float(shutdown_grace_seconds)

    def run_matrix(self, specs):
        """Run trials strictly sequentially; never reuse a ROS master."""
        return [self.run_trial(spec) for spec in specs]

    def run_trial(self, spec):
        try:
            self.process_manager.assert_no_matching_processes(
                ROS_PROCESS_PATTERNS
            )
        except ProcessPrecheckError as error:
            return self._record(spec, EXIT_LAUNCH_FAILED,
                                "precheck_failed: {}".format(error))
        try:
            process = self.process_manager.popen(roslaunch_command(spec))
        except OSError as error:
            return self._record(spec, EXIT_LAUNCH_FAILED,
                                "roslaunch_spawn_failed: {}".format(error))
        frozen_deadline = (
            self.clock.monotonic() + self.registration_timeout_seconds
        )
        if not self.frozen_watch_factory(spec).wait(frozen_deadline):
            self.process_manager.send_signal(process, signal.SIGINT)
            teardown_clean = self._reap(process)
            quiet = self._system_is_quiet()
            return self._record(
                spec, EXIT_REGISTRATION_FAILED, "frozen_timeout",
                {"teardown_clean": teardown_clean and quiet},
            )
        terminal_deadline = self.clock.monotonic() + spec.timeout_seconds
        while True:
            if self.clock.monotonic() >= terminal_deadline:
                self.process_manager.send_signal(process, signal.SIGINT)
                teardown_clean = self._reap(process)
                return self._record(
                    spec, EXIT_TIMEOUT, "terminal_state_timeout",
                    {"teardown_clean": teardown_clean and
                     self._system_is_quiet()},
                )
            document = self.result_reader.read(spec)
            if document is not None:
                break
            exit_code = self.process_manager.poll(process)
            if exit_code is not None:
                return self._record(
                    spec, EXIT_LAUNCH_FAILED,
                    "roslaunch_exited_early",
                    {"roslaunch_exit_code": exit_code,
                     "teardown_clean": self._system_is_quiet()},
                )
            self.clock.sleep(self.poll_interval_seconds)
        classification, failure_code = classify_trial_row(document["result"])
        flush_requested = False
        if self.flush_recorder is not None:
            self.flush_recorder(spec)
            flush_requested = True
        self.process_manager.send_signal(process, signal.SIGINT)
        teardown_clean = self._reap(process)
        quiet = self._system_is_quiet()
        return self._record(
            spec, classification,
            "recorder_status:{}".format(document["result"]["status"]),
            {
                "failure_code": failure_code,
                "success": bool(document["result"].get("success")),
                "flush_requested": flush_requested,
                "teardown_clean": teardown_clean and quiet,
            },
        )

    def _reap(self, process):
        try:
            self.process_manager.wait(process, self.shutdown_grace_seconds)
        except TimeoutError:
            self.process_manager.kill(process)
            self.process_manager.wait(process, self.shutdown_grace_seconds)
        return True

    def _system_is_quiet(self):
        try:
            self.process_manager.assert_no_matching_processes(
                ROS_PROCESS_PATTERNS
            )
        except ProcessPrecheckError:
            return False
        return True

    def _record(self, spec, exit_name, reason, extra=None):
        document = {
            "trial_id": spec.trial_id,
            "seed": spec.seed,
            "exit": exit_name,
            "reason": reason,
        }
        if extra:
            document.update(extra)
        self.classification_writer.write(spec, document)
        return exit_name


class WallClock:
    """Monotonic wall-clock implementation of the clock interface."""

    def monotonic(self):
        import time

        return time.monotonic()

    def sleep(self, seconds):
        import time

        time.sleep(seconds)


class SystemProcessManager:
    """Real process manager backed by /proc scanning and subprocess."""

    def assert_no_matching_processes(self, patterns):
        running = matching_processes(patterns, scan_running_processes())
        if running:
            raise ProcessPrecheckError(
                "matching processes still running: pids {}".format(running)
            )

    def popen(self, argv):
        import subprocess

        return subprocess.Popen(argv)

    def poll(self, process):
        return process.poll()

    def send_signal(self, process, sig):
        process.send_signal(sig)

    def wait(self, process, timeout):
        return process.wait(timeout=timeout)

    def kill(self, process):
        process.kill()


class RosFrozenWatch:
    """Watch the latched registration frozen topic; ROS imports stay lazy."""

    def __init__(self, spec, topic="/air_ground/registration/frozen"):
        import rospy
        from std_msgs.msg import Bool

        self._frozen = False
        rospy.init_node(
            "run_experiment_matrix_{}".format(safe_trial_file_stem(spec.trial_id)),
            anonymous=True,
        )
        rospy.Subscriber(topic, Bool, self._callback, queue_size=1)

    def _callback(self, message):
        if message.data:
            self._frozen = True

    def wait(self, deadline):
        import rospy

        while not self._frozen and rospy.get_rostime() is not None:
            import time

            if time.monotonic() >= deadline:
                return False
            time.sleep(0.1)
        return True


def load_matrix_config(path):
    import yaml

    with open(path) as stream:
        return yaml.safe_load(stream)


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(
        description="Run the one-shot cold-start registration matrix."
    )
    parser.add_argument("--config", required=True,
                        help="Path to one_shot_matrix.yaml")
    parser.add_argument("--trials", default="",
                        help="Comma-separated trial ids to run (default: all)")
    parser.add_argument("--output-root", default="",
                        help="Redirect every per-trial directory under this "
                             "fresh root instead of the frozen config root")
    parser.add_argument("--force", action="store_true",
                        help="Delete stale per-trial result directories "
                             "instead of refusing to run")
    args = parser.parse_args(argv)

    specs = expand_matrix(load_matrix_config(args.config))
    if args.trials:
        wanted = {name.strip() for name in args.trials.split(",") if name.strip()}
        specs = [spec for spec in specs if spec.trial_id in wanted]
    if args.output_root:
        specs = apply_output_root(specs, args.output_root)

    status, conflicts = evaluate_stale_outputs(specs, force=args.force)
    if status == "conflict":
        print(
            "Refusing to run: stale results from a previous invocation "
            "exist for {}.".format(", ".join(conflicts))
        )
        print(
            "Use a fresh root per matrix invocation (--output-root) or "
            "rerun with --force to delete those directories first."
        )
        return 3

    runner = TrialRunner(
        SystemProcessManager(),
        WallClock(),
        RosFrozenWatch,
        JsonResultReader(),
        JsonClassificationWriter(),
        flush_recorder=lambda spec: None,
    )
    classifications = runner.run_matrix(specs)
    for spec, classification in zip(specs, classifications):
        print("{} {}".format(spec.trial_id, classification))
    return 0 if all(name == EXIT_PASS for name in classifications) else 2


if __name__ == "__main__":
    raise SystemExit(main())
