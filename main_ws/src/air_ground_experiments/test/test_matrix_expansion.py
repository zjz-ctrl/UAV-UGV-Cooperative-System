#!/usr/bin/env python3
"""Deterministic one-shot matrix expansion and cold-start trial lifecycle."""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
CONFIG = PACKAGE / "config"
BRINGUP = PACKAGE.parents[1] / "src" / "air_ground_bringup"

sys.path.insert(0, str(SCRIPTS))

from run_experiment_matrix import (  # noqa: E402
    EXIT_LAUNCH_FAILED,
    EXIT_REGISTRATION_FAILED,
    ProcessPrecheckError,
    TrialRunner,
    TrialSpec,
    expand_matrix,
)

import signal  # noqa: E402


BOUNDS = {
    "uav_x": [-4.0, -2.0],
    "uav_y": [-2.0, 2.0],
    "uav_yaw_deg": [-180.0, 180.0],
    "ugv_heading_offset_deg": [-180.0, 180.0],
    "uav_frame_xy_m": [-3.0, 3.0],
    "uav_frame_yaw_deg": [-45.0, 45.0],
    "ugv_frame_xy_m": [-3.0, 3.0],
    "ugv_frame_yaw_deg": [-45.0, 45.0],
}


def load_fixture():
    return {
        "mode": "one_shot",
        "launch": {
            "package": "air_ground_bringup",
            "file": "air_ground_inspection_experiment.launch",
        },
        "seeds": list(range(1000, 1030)),
        "registration_dx_m": 0.60,
        "registration_dy_m": 0.0,
        "timeout_seconds": 180.0,
        "epoch_seconds": 0.0,
        "output_root": "/tmp/air_ground_experiments/matrix_one_shot",
        "bounds": dict(BOUNDS),
        "drift": "zero",
    }


class MatrixExpansionTest(unittest.TestCase):
    def test_fixture_yields_thirty_sorted_trials_with_frozen_seeds(self):
        trials = expand_matrix(load_fixture())

        self.assertEqual(len(trials), 30)
        self.assertEqual(trials[0].seed, 1000)
        self.assertEqual(trials[-1].trial_id, "one_shot-0029")

    def test_trials_are_sorted_by_trial_id_regardless_of_seed_order(self):
        fixture = load_fixture()
        shuffled = dict(fixture, seeds=list(reversed(fixture["seeds"])))

        trials = expand_matrix(shuffled)

        identifiers = [trial.trial_id for trial in trials]
        self.assertEqual(identifiers, sorted(identifiers))
        self.assertEqual(identifiers[0], "one_shot-0000")
        self.assertEqual(trials[0].seed, 1000)

    def test_sampled_poses_stay_within_declared_bounds(self):
        trials = expand_matrix(load_fixture())

        for trial in trials:
            args = trial.launch_args
            self.assertGreaterEqual(float(args["uav_x"]), -4.0)
            self.assertLessEqual(float(args["uav_x"]), -2.0)
            self.assertGreaterEqual(float(args["uav_y"]), -2.0)
            self.assertLessEqual(float(args["uav_y"]), 2.0)
            self.assertGreaterEqual(float(args["uav_yaw"]), -180.0)
            self.assertLessEqual(float(args["uav_yaw"]), 180.0)

    def test_registration_altitude_is_pinned_for_frozen_baseline_trials(self):
        fixture = load_fixture()
        fixture["registration_altitude_m"] = 1.8

        explicit = expand_matrix(fixture)
        for trial in explicit:
            self.assertEqual(trial.launch_args["registration_altitude"], "1.8")

        legacy = expand_matrix(load_fixture())
        for trial in legacy:
            # Historical one-shot trials ran at the script default 1.5; the
            # runner must keep pinning it explicitly even when the launch
            # default moves on for manual runs.
            self.assertEqual(trial.launch_args["registration_altitude"], "1.5")

    def test_ugv_spawns_at_the_uav_body_relative_registration_waypoint(self):
        import math

        fixture = load_fixture()
        dx, dy = fixture["registration_dx_m"], fixture["registration_dy_m"]
        trials = expand_matrix(fixture)

        for trial in trials:
            args = trial.launch_args
            uav_yaw_rad = math.radians(float(args["uav_yaw"]))
            expected_x = float(args["uav_x"]) + math.cos(uav_yaw_rad) * dx
            expected_y = float(args["uav_y"]) + math.sin(uav_yaw_rad) * dx
            self.assertAlmostEqual(float(args["ugv_x"]), expected_x, places=9)
            self.assertAlmostEqual(float(args["ugv_y"]), expected_y, places=9)
            unnormalized = (
                float(args["uav_yaw"]) + trial.sampled_parameters["ugv_heading_offset_deg"]
            )
            wrapped = (unnormalized + 180.0) % 360.0 - 180.0
            self.assertAlmostEqual(float(args["ugv_yaw"]), wrapped, places=9)

    def test_expansion_is_reproducible_field_for_field(self):
        first = expand_matrix(load_fixture())
        second = expand_matrix(load_fixture())

        for left, right in zip(first, second):
            self.assertEqual(left.trial_id, right.trial_id)
            self.assertEqual(left.seed, right.seed)
            self.assertEqual(left.launch_args, right.launch_args)
            self.assertEqual(left.sampled_parameters, right.sampled_parameters)
            self.assertEqual(left.timeout_seconds, right.timeout_seconds)
            self.assertEqual(left.output_directory, right.output_directory)

    def test_distinct_trial_seeds_produce_distinct_parameters(self):
        trials = expand_matrix(load_fixture())

        parameter_sets = {
            trial.seed: tuple(sorted(trial.sampled_parameters.items()))
            for trial in trials
        }
        self.assertEqual(len(parameter_sets), 30)
        self.assertGreater(
            len(set(parameter_sets.values())), 1,
            "distinct seeds must produce distinct sampled parameters",
        )

    def test_launch_args_pin_one_shot_zero_drift_and_per_trial_outputs(self):
        fixture = load_fixture()
        trials = expand_matrix(fixture)

        for trial in trials:
            args = trial.launch_args
            self.assertEqual(args["registration_mode"], "one_shot")
            self.assertEqual(args["use_visual_frame_yaw"], "true")
            self.assertEqual(args["translational_drift_rate"], "0.0")
            self.assertEqual(args["yaw_drift_rate"], "0.0")
            self.assertEqual(args["seed"], str(trial.seed))
            self.assertEqual(args["trial_id"], trial.trial_id)
            self.assertEqual(float(args["timeout_seconds"]), 180.0)
            self.assertEqual(float(args["epoch_seconds"]), 0.0)
            self.assertTrue(
                trial.output_directory.startswith(fixture["output_root"])
            )
            self.assertTrue(trial.output_directory.endswith(trial.trial_id))

    def test_frame_perturbation_offsets_respect_their_bounds(self):
        import math

        trials = expand_matrix(load_fixture())

        for trial in trials:
            uav_offsets = yaml.safe_load(trial.launch_args["uav_initial_xyyaw"])
            ugv_offsets = yaml.safe_load(trial.launch_args["ugv_initial_xyyaw"])
            for dx, dy, dyaw_rad in (
                tuple(uav_offsets[:2]) + (uav_offsets[2],),
                tuple(ugv_offsets[:2]) + (ugv_offsets[2],),
            ):
                self.assertGreaterEqual(dx, -3.0)
                self.assertLessEqual(dx, 3.0)
                self.assertGreaterEqual(dy, -3.0)
                self.assertLessEqual(dy, 3.0)
                self.assertGreaterEqual(dyaw_rad, math.radians(-45.0))
                self.assertLessEqual(dyaw_rad, math.radians(45.0))

    def test_frozen_yaml_matrix_matches_the_brief_fixture(self):
        with (CONFIG / "one_shot_matrix.yaml").open() as stream:
            config = yaml.safe_load(stream)

        self.assertEqual(config["mode"], "one_shot")
        self.assertEqual(config["drift"], "zero")
        self.assertEqual(config["seeds"], list(range(1000, 1030)))
        self.assertEqual(config["bounds"], BOUNDS)
        self.assertEqual(
            expand_matrix(config), expand_matrix(load_fixture())
        )


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += float(seconds)


class FakeProcess:
    def __init__(self):
        self.signals = []
        self.killed = False

    def __repr__(self):
        return "FakeProcess"


class FakeProcessManager:
    def __init__(self, precheck_error=None, popen_error=None, exit_code=None):
        self.precheck_calls = []
        self.spawned = []
        self.reaped = []
        self.precheck_error = precheck_error
        self.popen_error = popen_error
        self.exit_code = exit_code

    def poll(self, process):
        return self.exit_code

    def assert_no_matching_processes(self, patterns):
        self.precheck_calls.append(tuple(patterns))
        if self.precheck_error is not None:
            raise ProcessPrecheckError(self.precheck_error)

    def popen(self, argv):
        if self.popen_error is not None:
            raise self.popen_error
        process = FakeProcess()
        self.spawned.append((argv, process))
        return process

    def send_signal(self, process, sig):
        process.signals.append(sig)

    def wait(self, process, timeout):
        self.reaped.append(process)
        return 0

    def kill(self, process):
        process.killed = True


class AlwaysFrozenWatch:
    def __init__(self, frozen):
        self.frozen = frozen
        self.deadlines = []

    def wait(self, deadline):
        self.deadlines.append(deadline)
        return self.frozen


class FrozenWatchFactory:
    def __init__(self, frozen=True):
        self.frozen = frozen
        self.watches = []

    def __call__(self, spec):
        watch = AlwaysFrozenWatch(self.frozen)
        self.watches.append(watch)
        return watch


class FakeResultReader:
    def __init__(self, documents=None):
        self.documents = list(documents or [])
        self.reads = 0

    def read(self, spec):
        self.reads += 1
        if self.documents:
            return self.documents.pop(0)
        return None


class RecordingClassificationWriter:
    def __init__(self):
        self.documents = []

    def write(self, spec, document):
        self.documents.append((spec.trial_id, dict(document)))
        return "classification.json"


def make_runner_spec():
    return expand_matrix(load_fixture())[0]


class TrialLifecycleTest(unittest.TestCase):
    def test_precheck_failure_fails_the_launch_without_spawning_roslaunch(self):
        manager = FakeProcessManager(precheck_error="rosmaster is running")
        clock = FakeClock()
        writer = RecordingClassificationWriter()
        runner = TrialRunner(
            manager,
            clock,
            FrozenWatchFactory(),
            FakeResultReader(),
            writer,
        )

        classification = runner.run_trial(make_runner_spec())

        self.assertEqual(classification, EXIT_LAUNCH_FAILED)
        self.assertEqual(manager.spawned, [])
        self.assertEqual(len(manager.precheck_calls), 1)
        trial_id, document = writer.documents[0]
        self.assertEqual(trial_id, "one_shot-0000")
        self.assertEqual(document["exit"], "LAUNCH_FAILED")

    def test_spawn_failure_fails_the_launch(self):
        manager = FakeProcessManager(popen_error=OSError("no roslaunch"))
        writer = RecordingClassificationWriter()
        runner = TrialRunner(
            manager, FakeClock(), FrozenWatchFactory(),
            FakeResultReader(), writer,
        )

        classification = runner.run_trial(make_runner_spec())

        self.assertEqual(classification, EXIT_LAUNCH_FAILED)
        self.assertEqual(manager.spawned, [])
        self.assertEqual(writer.documents[0][1]["exit"], "LAUNCH_FAILED")

    def test_frozen_phase_timeout_registers_failure_and_tears_down(self):
        import signal

        manager = FakeProcessManager()
        writer = RecordingClassificationWriter()
        watches = FrozenWatchFactory(frozen=False)
        runner = TrialRunner(
            manager, FakeClock(), watches, FakeResultReader(), writer,
            registration_timeout_seconds=120.0,
        )
        spec = make_runner_spec()

        classification = runner.run_trial(spec)

        self.assertEqual(classification, EXIT_REGISTRATION_FAILED)
        argv, process = manager.spawned[0]
        self.assertIn(signal.SIGINT, process.signals)
        self.assertIn(process, manager.reaped)
        # Teardown re-check: no ROS master reuse between trials.
        self.assertEqual(len(manager.precheck_calls), 2)
        document = writer.documents[0][1]
        self.assertEqual(document["exit"], "REGISTRATION_FAILED")
        self.assertEqual(document["reason"], "frozen_timeout")
        self.assertTrue(watches.watches[0].deadlines[0] <= 120.0)

    def _run_with_row(self, row):
        return run_trial_with_row(row)


def run_trial_with_row(row):
    manager = FakeProcessManager()
    writer = RecordingClassificationWriter()
    recorder_calls = []
    runner = TrialRunner(
        manager,
        FakeClock(),
        FrozenWatchFactory(frozen=True),
        FakeResultReader([{"result": row, "metadata": {}}]),
        writer,
        flush_recorder=lambda spec: recorder_calls.append(spec.trial_id),
    )
    spec = make_runner_spec()
    classification = runner.run_trial(spec)
    _, process = manager.spawned[0]
    return classification, writer.documents[0][1], process, recorder_calls


class TerminalMappingTest(unittest.TestCase):
    def _run_with_row(self, row):
        return run_trial_with_row(row)

    def test_completed_success_row_maps_to_pass(self):
        row = {"trial_id": "one_shot-0000", "seed": 1000,
               "status": "COMPLETED", "success": True, "failure_code": ""}
        classification, document, process, flushed = self._run_with_row(row)

        self.assertEqual(classification, "PASS")
        self.assertEqual(document["exit"], "PASS")
        self.assertEqual(process.signals, [signal.SIGINT])
        self.assertEqual(flushed, ["one_shot-0000"])

    def test_mission_phase_failures_map_to_mission_failed(self):
        row = {"trial_id": "one_shot-0000", "seed": 1000,
               "status": "FAILED", "success": False,
               "failure_code": "MISSION_APPROACH"}
        classification, _, _, _ = self._run_with_row(row)

        self.assertEqual(classification, "MISSION_FAILED")

    def test_registration_domain_failures_map_to_registration_failed(self):
        for code in ("MISSION_REGISTRATION", "INCOMPLETE_TRUTH_SYNC",
                     "ANOMALY_TRUTH_UNAVAILABLE"):
            row = {"trial_id": "one_shot-0000", "seed": 1000,
                   "status": "FAILED", "success": False, "failure_code": code}
            classification, _, _, _ = self._run_with_row(row)

            self.assertEqual(classification, "REGISTRATION_FAILED", code)

    def test_recorder_timeout_rows_map_to_timeout(self):
        row = {"trial_id": "one_shot-0000", "seed": 1000,
               "status": "TIMEOUT", "success": False,
               "failure_code": "TRIAL_TIMEOUT"}
        classification, _, _, _ = self._run_with_row(row)

        self.assertEqual(classification, "TIMEOUT")

    def test_outside_radius_and_unknown_codes_default_to_mission_failed(self):
        for code in ("OUTSIDE_SUCCESS_RADIUS", "SOMETHING_NEW"):
            row = {"trial_id": "one_shot-0000", "seed": 1000,
                   "status": "FAILED", "success": False, "failure_code": code}
            classification, _, _, _ = self._run_with_row(row)

            self.assertEqual(classification, "MISSION_FAILED", code)

    def test_runner_side_deadline_backstop_maps_to_timeout(self):
        manager = FakeProcessManager()
        writer = RecordingClassificationWriter()
        runner = TrialRunner(
            manager, FakeClock(), FrozenWatchFactory(frozen=True),
            FakeResultReader([]), writer, poll_interval_seconds=10.0,
        )
        spec = make_runner_spec()

        classification = runner.run_trial(spec)

        self.assertEqual(classification, "TIMEOUT")
        _, process = manager.spawned[0]
        self.assertEqual(process.signals, [signal.SIGINT])
        document = writer.documents[0][1]
        self.assertEqual(document["reason"], "terminal_state_timeout")

    def test_roslaunch_exit_before_result_maps_to_launch_failed(self):
        manager = FakeProcessManager(exit_code=1)
        writer = RecordingClassificationWriter()
        runner = TrialRunner(
            manager, FakeClock(), FrozenWatchFactory(frozen=True),
            FakeResultReader([]), writer,
        )

        classification = runner.run_trial(make_runner_spec())

        self.assertEqual(classification, "LAUNCH_FAILED")
        self.assertEqual(
            writer.documents[0][1]["reason"], "roslaunch_exited_early"
        )

    def test_classification_writer_creates_trial_directory_and_exit_json(self):
        import json
        import tempfile

        from run_experiment_matrix import JsonClassificationWriter

        spec = make_runner_spec()
        document = {"trial_id": spec.trial_id, "seed": spec.seed,
                    "exit": "PASS", "reason": "recorder_status:COMPLETED"}
        with tempfile.TemporaryDirectory() as root:
            trial_spec = TrialSpec(
                spec.trial_id, spec.seed, spec.launch_args,
                spec.timeout_seconds, str(Path(root) / "one_shot-0000"),
            )
            path = JsonClassificationWriter().write(trial_spec, document)

            parsed = json.loads(Path(path).read_text())

        self.assertEqual(parsed["exit"], "PASS")
        self.assertEqual(parsed["trial_id"], "one_shot-0000")
        self.assertEqual(Path(path).parent.name, "one_shot-0000")

    def test_matrix_runs_trials_sequentially_without_reusing_a_master(self):
        fixture = load_fixture()
        specs = expand_matrix(fixture)[:3]
        success = {"result": {"trial_id": "", "seed": 0,
                              "status": "COMPLETED", "success": True,
                              "failure_code": ""}, "metadata": {}}
        manager = FakeProcessManager()
        writer = RecordingClassificationWriter()
        runner = TrialRunner(
            manager,
            FakeClock(),
            FrozenWatchFactory(frozen=True),
            FakeResultReader([dict(success) for _ in specs]),
            writer,
        )

        classifications = runner.run_matrix(specs)

        self.assertEqual(classifications, ["PASS", "PASS", "PASS"])
        # One pre-check before and one quiet-system verification after every
        # trial: no ROS master or simulator is ever reused between trials.
        self.assertEqual(len(manager.precheck_calls), 2 * len(specs))
        for index, (argv, _) in enumerate(manager.spawned):
            self.assertIn("trial_id:=one_shot-{:04d}".format(index), argv)
            self.assertIn("registration_mode:=one_shot", argv)

    def test_every_spawned_argv_carries_the_per_trial_output_directory(self):
        specs = expand_matrix(load_fixture())[:3]
        success = {"result": {"trial_id": "", "seed": 0,
                              "status": "COMPLETED", "success": True,
                              "failure_code": ""}, "metadata": {}}
        manager = FakeProcessManager()
        runner = TrialRunner(
            manager,
            FakeClock(),
            FrozenWatchFactory(frozen=True),
            FakeResultReader([dict(success) for _ in specs]),
            RecordingClassificationWriter(),
        )

        runner.run_matrix(specs)

        for spec, (argv, _) in zip(specs, manager.spawned):
            self.assertIn(
                "output_directory:={}".format(spec.output_directory), argv
            )

    def test_proc_scanner_detects_matching_ros_processes(self):
        from run_experiment_matrix import matching_processes

        running = {
            "10": "rosmaster __cmd:=roscore",
            "11": "/usr/bin/python3 /path/to/roslaunch demo.launch",
            "12": "gzserver --verbose ego.world",
            "13": "gzclient",
            "14": "px4 -s etc/init.d-posix/rcS",
            "20": "gedit notes.txt",
            "21": "python3 test_matrix_expansion.py",
        }

        matched = matching_processes(
            ("roslaunch", "rosmaster", "gzserver", "gzclient", "px4"), running
        )

        self.assertEqual(sorted(matched), ["10", "11", "12", "13", "14"])

    def test_result_reader_parses_recorder_json_or_returns_none(self):
        import json
        import tempfile

        from run_experiment_matrix import JsonResultReader

        row = {"trial_id": "one_shot-0000", "seed": 1000,
               "status": "COMPLETED", "success": True, "failure_code": ""}
        spec = make_runner_spec()
        with tempfile.TemporaryDirectory() as root:
            trial_dir = Path(root) / spec.trial_id
            trial_dir.mkdir()
            (trial_dir / "{}.json".format(spec.trial_id)).write_text(
                json.dumps({"result": row, "metadata": {}})
            )
            reader = JsonResultReader()

            missing = reader.read(
                TrialSpec(spec.trial_id, spec.seed, {}, 1.0,
                          str(Path(root) / "absent"))
            )
            found = reader.read(
                TrialSpec(spec.trial_id, spec.seed, {}, 1.0, str(trial_dir))
            )

        self.assertIsNone(missing)
        self.assertEqual(found["result"]["status"], "COMPLETED")


class RunnerSourceAuditTest(unittest.TestCase):
    def test_module_imports_stay_ros_free_for_pure_execution(self):
        import ast

        tree = ast.parse((SCRIPTS / "run_experiment_matrix.py").read_text())
        for node in tree.body:
            names = []
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            for name in names:
                self.assertFalse(
                    name.split(".")[0] in ("rospy", "rosbag", "tf"),
                    "module-level ROS import: {}".format(name),
                )

    def test_main_glue_wires_freshness_guard_and_output_override(self):
        source = (SCRIPTS / "run_experiment_matrix.py").read_text()
        self.assertIn('"--force"', source)
        self.assertIn('"--output-root"', source)
        self.assertIn("evaluate_stale_outputs(specs", source)
        self.assertIn("apply_output_root(specs", source)
        self.assertIn("--output-root", source)
        self.assertIn("--force", source)


def spec_with_populated_directory(root, trial_id, filenames):
    from run_experiment_matrix import TrialSpec as ImplementationSpec
    directory = Path(root) / trial_id
    directory.mkdir(parents=True, exist_ok=True)
    for name in filenames:
        (directory / name).write_text("{}\n")
    return ImplementationSpec(
        trial_id, 1000, {}, 1.0, str(directory)
    )


class StaleOutputGuardTest(unittest.TestCase):
    def test_preflight_rejects_directories_with_stale_results(self):
        import tempfile

        from run_experiment_matrix import evaluate_stale_outputs

        with tempfile.TemporaryDirectory() as root:
            recorder_stale = spec_with_populated_directory(
                root, "one_shot-0000", ["one_shot-0000.json"]
            )
            exit_stale = spec_with_populated_directory(
                root, "one_shot-0001", ["exit.json"]
            )
            fresh = spec_with_populated_directory(root, "one_shot-0002", [])

            status, conflicts = evaluate_stale_outputs(
                [recorder_stale, exit_stale, fresh]
            )

        self.assertEqual(status, "conflict")
        self.assertEqual(conflicts, ["one_shot-0000", "one_shot-0001"])

    def test_force_clears_stale_trial_directories_entirely(self):
        import os
        import tempfile

        from run_experiment_matrix import evaluate_stale_outputs

        with tempfile.TemporaryDirectory() as root:
            stale = spec_with_populated_directory(
                root, "one_shot-0000",
                ["one_shot-0000.json", "exit.json", "trials.csv"],
            )

            status, cleared = evaluate_stale_outputs([stale], force=True)

            self.assertEqual(status, "cleared")
            self.assertEqual(cleared, ["one_shot-0000"])
            self.assertFalse(os.path.exists(stale.output_directory))
            self.assertEqual(evaluate_stale_outputs([stale])[0], "ok")

    def test_output_root_override_redirects_every_trial_directory(self):
        import tempfile

        from run_experiment_matrix import apply_output_root

        original = expand_matrix(load_fixture())[:2]
        with tempfile.TemporaryDirectory() as root:
            overridden = apply_output_root(original, root)

        for source, redirected in zip(original, overridden):
            self.assertEqual(redirected.trial_id, source.trial_id)
            self.assertEqual(
                redirected.output_directory,
                str(Path(root) / source.trial_id),
            )
            # The frozen config output roots stay untouched.
            self.assertEqual(
                source.output_directory,
                "/tmp/air_ground_experiments/matrix_one_shot/{}".format(
                    source.trial_id
                ),
            )

    def test_output_root_override_syncs_argv_with_the_reader_directory(self):
        import tempfile

        from run_experiment_matrix import (
            apply_output_root,
            roslaunch_command,
        )

        specs = expand_matrix(load_fixture())[:2]
        with tempfile.TemporaryDirectory() as root:
            redirected = apply_output_root(specs, root)

            for spec in redirected:
                expected = str(Path(root) / spec.trial_id)
                self.assertEqual(
                    spec.launch_args["output_directory"], expected
                )
                self.assertIn(
                    "output_directory:={}".format(expected),
                    roslaunch_command(spec),
                )


def named_child(parent, tag, name):
    child = next(
        (child for child in parent.findall(tag) if child.get("name") == name),
        None,
    )
    if child is None:
        raise AssertionError("missing {} named {!r}".format(tag, name))
    return child


EXPERIMENT_LAUNCH = BRINGUP / "launch" / "air_ground_inspection_experiment.launch"


class IntegrationLaunchAuditTest(unittest.TestCase):
    def root(self):
        return ET.parse(EXPERIMENT_LAUNCH).getroot()

    def test_launch_declares_one_shot_mode_with_visual_yaw_enabled(self):
        root = self.root()

        mode = named_child(root, "arg", "registration_mode")
        self.assertEqual(mode.get("default"), "one_shot")
        visual_yaw = named_child(root, "arg", "use_visual_frame_yaw")
        self.assertEqual(visual_yaw.get("default"), "true")

    def test_perturbation_layer_sits_between_raw_sources_and_research(self):
        root = self.root()

        include = next(
            child for child in root.findall("include")
            if child.get("file", "").endswith("/launch/frame_perturbation.launch")
        )
        wired = {
            arg.get("name"): arg.get("value")
            for arg in include.findall("arg")
        }
        for name in ("seed", "uav_initial_xyyaw", "ugv_initial_xyyaw",
                     "translational_drift_rate", "yaw_drift_rate",
                     "epoch_seconds", "output_directory", "trial_id"):
            self.assertEqual(wired.get(name), "$(arg {})".format(name), name)

        uav_sim = next(
            child for child in root.findall("include")
            if child.get("file", "").endswith("/launch/uav_sitl.launch")
        )
        for leaf, parent in (("x", "uav_x"), ("y", "uav_y"), ("yaw", "uav_yaw")):
            self.assertEqual(
                named_child(uav_sim, "arg", leaf).get("value"),
                "$(arg {})".format(parent),
            )
        ugv_spawn = next(
            child for child in root.findall("include")
            if child.get("file", "").endswith("/launch/spawn_ugv.launch")
        )
        for leaf, parent in (("x", "ugv_x"), ("y", "ugv_y"), ("yaw", "ugv_yaw")):
            self.assertEqual(
                named_child(ugv_spawn, "arg", leaf).get("value"),
                "$(arg {})".format(parent),
            )

    def test_research_nodes_consume_experiment_streams_and_route_commands(self):
        root = self.root()
        nodes = {node.get("name"): node for node in root.iter("node")}

        registration = nodes["takeoff_registration"]
        for param, value in (
            ("uav_odom_topic", "/air_ground_experiment/uav/odom"),
            ("ugv_odom_topic", "/air_ground_experiment/ugv/odom"),
            ("observation_topic", "/air_ground_experiment/charuco/observation"),
            ("use_visual_frame_yaw", "$(arg use_visual_frame_yaw)"),
            ("registration_mode", "$(arg registration_mode)"),
        ):
            element = registration.find("param[@name='{}']".format(param))
            self.assertIsNotNone(element, param)
            self.assertEqual(element.get("value"), value)

        mission = nodes["uav_sphere_mission"]
        remaps = {
            remap.get("from"): remap.get("to") for remap in mission.findall("remap")
        }
        self.assertEqual(
            remaps["/iris_0/mavros/local_position/odom"],
            "/air_ground_experiment/uav/odom",
        )
        self.assertEqual(
            remaps["/ugv_0/odom"], "/air_ground_experiment/ugv/odom"
        )
        self.assertEqual(
            remaps["/iris_0/position_cmd"],
            "/air_ground_experiment/iris_0/position_cmd",
        )

        takeoff = nodes["auto_takeoff_trigger"]
        trigger_remaps = {
            remap.get("from"): remap.get("to")
            for remap in takeoff.findall("remap")
        }
        self.assertEqual(
            trigger_remaps["position_cmd"],
            "/air_ground_experiment/iris_0/position_cmd",
        )
        self.assertEqual(
            trigger_remaps["mavros/local_position/odom"],
            "/air_ground_experiment/uav/odom",
        )
        self.assertIn("cxr_egoctrl", nodes)

    def test_launch_introduces_no_truth_topics_outside_the_include(self):
        root = ET.tostring(self.root(), encoding="unicode")

        self.assertNotIn("/air_ground_experiment/truth/", root)
        self.assertIn("frame_perturbation.launch", root)

    def test_every_emitted_launch_arg_is_declared_by_the_experiment_launch(self):
        declared = {
            arg.get("name") for arg in self.root().findall("arg")
        }
        emitted = set(expand_matrix(load_fixture())[0].launch_args)

        undeclared = sorted(emitted - declared)
        self.assertEqual(
            undeclared, [],
            "runner emits launch args the experiment launch does not "
            "declare; roslaunch would silently drop them",
        )

    def test_frozen_timeout_budget_reaches_the_recorder(self):
        perturbation = ET.parse(
            PACKAGE / "launch" / "frame_perturbation.launch"
        ).getroot()
        recorder = next(
            node for node in perturbation.findall("node")
            if node.get("name") == "experiment_recorder"
        )

        budget_arg = named_child(perturbation, "arg", "timeout_seconds")
        self.assertEqual(budget_arg.get("default"), "120.0")
        budget_param = recorder.find("param[@name='timeout_seconds']")
        self.assertIsNotNone(budget_param)
        self.assertEqual(budget_param.get("value"), "$(arg timeout_seconds)")

        include = next(
            child for child in self.root().findall("include")
            if child.get("file", "").endswith("/launch/frame_perturbation.launch")
        )
        forwarded = named_child(include, "arg", "timeout_seconds")
        self.assertEqual(forwarded.get("value"), "$(arg timeout_seconds)")


class BringupManifestAuditTest(unittest.TestCase):
    def test_bringup_declares_experiment_and_perception_dependencies(self):
        package_xml = ET.parse(
            BRINGUP / "package.xml"
        ).getroot()
        exec_depends = {
            element.text
            for element in package_xml
            if element.tag.endswith("exec_depend")
        }
        self.assertIn("air_ground_experiments", exec_depends)
        self.assertIn("air_ground_perception", exec_depends)
        build_depends = {
            element.text
            for element in package_xml
            if element.tag.endswith("build_depend")
        }
        self.assertIn("air_ground_experiments", build_depends)
        self.assertIn("air_ground_perception", build_depends)

        cmake = (BRINGUP / "CMakeLists.txt").read_text()
        find_start = cmake.index("find_package(catkin REQUIRED COMPONENTS")
        find_block = cmake[find_start:cmake.index(")", find_start)]
        self.assertIn("air_ground_experiments", find_block)
        self.assertIn("air_ground_perception", find_block)
        catkin_package_block = cmake[
            cmake.index("catkin_package("):
        ]
        self.assertIn("air_ground_experiments", catkin_package_block)
        self.assertIn("air_ground_perception", catkin_package_block)
        self.assertIn("install(DIRECTORY config launch rviz scripts", cmake)


if __name__ == "__main__":
    unittest.main()
