#!/usr/bin/env python3

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest


PHYSICAL_WORKSPACE = Path(__file__).resolve().parents[3]
SETUP = PHYSICAL_WORKSPACE / "src" / "air_ground_bringup" / "scripts" / "setup_mvp_env.sh"

CANONICAL_ROOT = os.environ.get("UAV_UGV_ROOT", str(Path.home() / "UAV-UGV_ws"))
CANONICAL_WORKSPACE = CANONICAL_ROOT + "/main_ws"
CANONICAL_PX4 = CANONICAL_ROOT + "/PX4-Autopilot"
FORBIDDEN_PATHS = ("/home/zjz/Ego_Planner_v2", "/home/zjz/PX4-Autopilot")
SEARCH_VARS = (
    "PATH",
    "ROS_PACKAGE_PATH",
    "CMAKE_PREFIX_PATH",
    "PKG_CONFIG_PATH",
    "ROSLISP_PACKAGE_DIRECTORIES",
    "PYTHONPATH",
    "GAZEBO_PLUGIN_PATH",
    "GAZEBO_MODEL_PATH",
    "LD_LIBRARY_PATH",
)


class SetupMvpEnvironmentTest(unittest.TestCase):
    def run_shell(self, command):
        environment = {
            "HOME": os.environ["HOME"],
            "PATH": os.environ["PATH"],
            "SHELL": "/bin/bash",
        }
        completed = subprocess.run(
            ["bash", "--noprofile", "--norc", "-c", command],
            cwd=str(PHYSICAL_WORKSPACE),
            env=environment,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        return completed.stdout.splitlines()

    @staticmethod
    def marked(lines, name):
        prefix = name + "="
        return next(line[len(prefix):] for line in lines if line.startswith(prefix))

    @staticmethod
    def physical_duplicates(entries):
        reals = [os.path.realpath(entry) for entry in entries]
        seen = {}
        for entry, real in zip(entries, reals):
            seen.setdefault(real, entry)
        duplicates = [entry for entry, real in zip(entries, reals)
                      if sum(1 for other in reals if other == real) > 1]
        return len(duplicates)

    def search_path_dump(self, extra_source_count=0, parent_prelude=""):
        source = self.source_command()
        sources = "{};\n".format(source) * (1 + max(0, int(extra_source_count)))
        printf_block = "; ".join(
            "printf '%s=%s\\n' \"{name}\" \"${{{name}}}\"".format(name=name)
            for name in SEARCH_VARS
        )
        lines = self.run_shell(
            "{prelude}{sources}{block}\n".format(
                prelude=parent_prelude,
                sources=sources,
                block=printf_block,
            )
        )
        return {name: self.marked(lines, name) for name in SEARCH_VARS}

    def source_command(self):
        canonical_setup = "{}/{}".format(
            CANONICAL_WORKSPACE,
            "/".join(SETUP.relative_to(PHYSICAL_WORKSPACE).parts),
        )
        return "source {} >/dev/null".format(shlex.quote(canonical_setup))

    def expected_ros_package_path(self):
        return [
            CANONICAL_PX4,
            CANONICAL_PX4 + "/Tools/sitl_gazebo",
            CANONICAL_WORKSPACE + "/src",
            "/opt/ros/noetic/share",
        ]

    def setUp(self):
        for required in (
            CANONICAL_WORKSPACE + "/devel/setup.bash",
            CANONICAL_PX4 + "/Tools/setup_gazebo.bash",
            CANONICAL_PX4 + "/build/px4_sitl_default",
        ):
            self.assertTrue(
                os.path.exists(required),
                "required deployment path missing in this checkout: {}".format(required),
            )

    def test_final_source_produces_canonical_ros_package_path(self):
        lines = self.run_shell(
            "{}; printf 'ROS_PACKAGE_PATH=%s\\n' \"$ROS_PACKAGE_PATH\"".format(
                self.source_command()
            )
        )

        self.assertEqual(
            self.marked(lines, "ROS_PACKAGE_PATH").split(":"),
            self.expected_ros_package_path(),
        )

    def test_no_exported_path_variable_mentions_forbidden_legacy_roots(self):
        lines = self.run_shell(
            "{0}; "
            "printf 'ROS_PACKAGE_PATH=%s\\n' \"$ROS_PACKAGE_PATH\"; "
            "printf 'GAZEBO_PLUGIN_PATH=%s\\n' \"$GAZEBO_PLUGIN_PATH\"; "
            "printf 'GAZEBO_MODEL_PATH=%s\\n' \"$GAZEBO_MODEL_PATH\"".format(
                self.source_command()
            )
        )

        for name in ("ROS_PACKAGE_PATH", "GAZEBO_PLUGIN_PATH", "GAZEBO_MODEL_PATH"):
            with self.subTest(name=name):
                self.assertNotIn(FORBIDDEN_PATHS[0], self.marked(lines, name))
                self.assertNotIn(FORBIDDEN_PATHS[1], self.marked(lines, name))

    def test_repeated_source_keeps_path_variables_duplicate_free(self):
        lines = self.run_shell(
            "{0}; {0}; "
            "printf 'ROS_PACKAGE_PATH=%s\\n' \"$ROS_PACKAGE_PATH\"; "
            "printf 'GAZEBO_PLUGIN_PATH=%s\\n' \"$GAZEBO_PLUGIN_PATH\"; "
            "printf 'GAZEBO_MODEL_PATH=%s\\n' \"$GAZEBO_MODEL_PATH\"; "
            "printf 'LD_LIBRARY_PATH=%s\\n' \"$LD_LIBRARY_PATH\"".format(
                self.source_command()
            )
        )

        for name in (
            "ROS_PACKAGE_PATH",
            "GAZEBO_PLUGIN_PATH",
            "GAZEBO_MODEL_PATH",
            "LD_LIBRARY_PATH",
        ):
            with self.subTest(name=name):
                entries = self.marked(lines, name).split(":")
                self.assertNotIn("", entries)
                self.assertEqual(len(entries), len(set(entries)))

        self.assertIn(
            CANONICAL_PX4 + "/build/px4_sitl_default/build_gazebo",
            self.marked(lines, "GAZEBO_PLUGIN_PATH").split(":"),
        )
        self.assertIn(
            CANONICAL_WORKSPACE + "/devel/lib",
            self.marked(lines, "GAZEBO_PLUGIN_PATH").split(":"),
        )
        self.assertIn(
            CANONICAL_PX4 + "/Tools/sitl_gazebo/models",
            self.marked(lines, "GAZEBO_MODEL_PATH").split(":"),
        )
        self.assertIn(
            CANONICAL_WORKSPACE + "/src/air_ground_ugv_gazebo/models",
            self.marked(lines, "GAZEBO_MODEL_PATH").split(":"),
        )

    def test_sourcing_wrapper_last_restores_px4_package_resolution(self):
        lines = self.run_shell(
            "{0}; source {1}; {0}; "
            "printf 'ROS_PACKAGE_PATH=%s\\n' \"$ROS_PACKAGE_PATH\"; "
            "printf 'PX4_PACKAGE=%s\\n' \"$(rospack find px4)\"; "
            "printf 'SITL_PACKAGE=%s\\n' \"$(rospack find mavlink_sitl_gazebo)\"; "
            "printf 'BRINGUP_PACKAGE=%s\\n' \"$(rospack find air_ground_bringup)\"".format(
                self.source_command(),
                shlex.quote(str(PHYSICAL_WORKSPACE / "devel" / "setup.bash")),
            )
        )

        self.assertEqual(
            self.marked(lines, "ROS_PACKAGE_PATH").split(":"),
            self.expected_ros_package_path(),
        )
        self.assertEqual(self.marked(lines, "PX4_PACKAGE"), CANONICAL_PX4)
        self.assertEqual(
            self.marked(lines, "SITL_PACKAGE"),
            CANONICAL_PX4 + "/Tools/sitl_gazebo",
        )
        self.assertEqual(
            self.marked(lines, "BRINGUP_PACKAGE"),
            CANONICAL_WORKSPACE + "/src/air_ground_bringup",
        )

    def test_source_succeeds_with_nounset_enabled(self):
        lines = self.run_shell(
            "set -u; {}; printf 'NOUNSET_SOURCE=ready\\n'".format(
                self.source_command()
            )
        )
        self.assertEqual(self.marked(lines, "NOUNSET_SOURCE"), "ready")

    def test_every_search_path_lists_the_workspace_once_physically(self):
        env = self.search_path_dump()
        for name, value in env.items():
            with self.subTest(variable=name):
                entries = [entry for entry in value.split(":") if entry]
                self.assertEqual(
                    self.physical_duplicates(entries), 0,
                    "duplicate physical directory in {}: {}".format(name, entries),
                )
                for forbidden in FORBIDDEN_PATHS:
                    offending = [
                        entry for entry in entries
                        if entry == forbidden or entry.startswith(forbidden + "/")
                    ]
                    self.assertEqual(
                        offending, [],
                        "forbidden legacy spelling in {}: {}".format(name, offending),
                    )

    def test_ten_consecutive_sources_converge_to_one_environment(self):
        once = self.search_path_dump(extra_source_count=0)
        ten = self.search_path_dump(extra_source_count=9)
        self.assertEqual(once, ten)

    def test_polluted_parent_environment_is_rebuilt_clean(self):
        prelude = (
            'export ROS_PACKAGE_PATH="/home/zjz/Ego_Planner_v2/src:'
            '/home/zjz/air_ground_cooperation/Ego_Planner_v2/src:'
            '/home/zjz/PX4-Autopilot:/opt/ros/noetic/share";'
            'export PYTHONPATH="/home/zjz/Ego_Planner_v2/devel/lib/python3/dist-packages:'
            '/home/zjz/Ego_Planner_v2/devel/lib/python3/dist-packages";'
            'export LD_LIBRARY_PATH="/home/zjz/Ego_Planner_v2/devel/lib:'
            '/home/zjz/air_ground_cooperation/Ego_Planner_v2/devel/lib:'
            '/opt/ros/noetic/lib";'
        )
        env = self.search_path_dump(parent_prelude=prelude)
        for name in ("ROS_PACKAGE_PATH", "PYTHONPATH", "LD_LIBRARY_PATH"):
            with self.subTest(variable=name):
                entries = [entry for entry in env[name].split(":") if entry]
                self.assertEqual(self.physical_duplicates(entries), 0)
                self.assertFalse(
                    any(entry.startswith(FORBIDDEN_PATHS[0] + "/") for entry in entries)
                )
        # The workspace artifacts survive the rebuild exactly once, spelled
        # through their canonical cooperation aliases.
        canonical_python = CANONICAL_WORKSPACE + "/devel/lib/python3/dist-packages"
        self.assertIn(canonical_python, env["PYTHONPATH"].split(":"))
        self.assertIn(CANONICAL_WORKSPACE + "/devel/lib", env["LD_LIBRARY_PATH"].split(":"))
        self.assertEqual(
            env["ROS_PACKAGE_PATH"].split(":"), self.expected_ros_package_path()
        )

    def test_roslaunch_launch_file_resolution_is_unique(self):
        checks = (
            ("air_ground_bringup", "air_ground_inspection_experiment.launch",
             "launch/air_ground_inspection_experiment.launch"),
        )
        python_lines = [
            "import os, traceback",
            "from roslib.packages import find_resource",
            "src_root = {root!r}".format(root=CANONICAL_WORKSPACE + "/src/"),
            "checks = {checks!r}".format(checks=checks),
            "try:",
            "    for pkg, res, rel in checks:",
            "        found = find_resource(pkg, res)",
            "        assert len(found) == 1, (pkg, res, found)",
            "        expected = os.path.realpath(src_root + pkg + '/' + rel)",
            "        assert found[0] == expected, (found[0], expected)",
            "    print('RESOLUTION_UNIQUE_OK')",
            "except Exception:",
            "    import traceback",
            "    traceback.print_exc()",
            "    print('RESOLUTION_UNIQUE_FAIL')",
        ]
        # Build the runner with the real snippet path, then execute it so the
        # wrapper's exports are visible to the python resolution check.
        with tempfile.TemporaryDirectory() as directory:
            snippet_path = Path(directory) / "resolution_check.py"
            snippet_path.write_text("\n".join(python_lines) + "\n")
            runner = Path(directory) / "runner.sh"
            runner.write_text(
                "{source}\n"
                "printf 'ROSPACK_PKG=%s\\n' \"$(rospack find air_ground_bringup)\"\n"
                "python3 {snippet}\n".format(
                    source=self.source_command(),
                    snippet=shlex.quote(str(snippet_path)),
                )
            )
            lines = self.run_shell("bash {}".format(shlex.quote(str(runner))))

        self.assertIn("RESOLUTION_UNIQUE_OK", lines)
        self.assertNotIn("RESOLUTION_UNIQUE_FAIL", lines)
        expected_package = CANONICAL_WORKSPACE + "/src/air_ground_bringup"
        self.assertEqual(self.marked(lines, "ROSPACK_PKG"), expected_package)

    def test_roslaunch_node_resolution_finds_each_devel_executable_once(self):
        checks = (
            ("air_ground_bringup", "uav_sphere_mission.py"),
            ("air_ground_bringup", "auto_takeoff_trigger.py"),
            ("cxr_egoctrl_v1", "cxr_egoctrl_v1"),
        )
        python_lines = [
            "import os",
            "from roslib.packages import find_node",
            "checks = {checks!r}".format(checks=checks),
            "ok = True",
            "for pkg, node in checks:",
            "    found = find_node(pkg, node)",
            "    print('NODE_COUNT={} {} {}'.format(pkg, node, len(found)))",
            "    expected = os.path.realpath({!r} + '/devel/lib/' + pkg + '/' + node)".format(
                CANONICAL_WORKSPACE
            ),
            "    ok = ok and found == [expected]",
            "print('NODE_RESOLUTION_OK' if ok else 'NODE_RESOLUTION_FAIL')",
        ]
        with tempfile.TemporaryDirectory() as directory:
            snippet_path = Path(directory) / "node_resolution_check.py"
            snippet_path.write_text("\n".join(python_lines) + "\n")
            lines = self.run_shell(
                "{source}; python3 {snippet}".format(
                    source=self.source_command(),
                    snippet=shlex.quote(str(snippet_path)),
                )
            )

        self.assertIn("NODE_RESOLUTION_OK", lines)
        self.assertFalse(
            os.access(
                CANONICAL_WORKSPACE
                + "/src/air_ground_bringup/scripts/uav_sphere_mission.py",
                os.X_OK,
            )
        )
        self.assertFalse(
            os.access(
                CANONICAL_WORKSPACE
                + "/src/air_ground_bringup/scripts/auto_takeoff_trigger.py",
                os.X_OK,
            )
        )

    def test_devel_marker_advertises_only_the_canonical_source_space(self):
        lines = self.run_shell(
            "{source}; printf 'CATKIN_MARKER=%s\\n' \"$(< {marker})\"".format(
                source=self.source_command(),
                marker=shlex.quote(CANONICAL_WORKSPACE + "/devel/.catkin"),
            )
        )
        self.assertEqual(
            self.marked(lines, "CATKIN_MARKER"),
            CANONICAL_WORKSPACE + "/src",
        )

    def test_catkin_prefix_chain_includes_one_canonical_devel_space(self):
        prelude = self.source_command() + "; "
        printf_cpp = "printf 'CMAKE_PREFIX_PATH=%s\\n' \"${CMAKE_PREFIX_PATH-}\"; "
        printf_lisp = (
            'if [ -n "${ROSLISP_PACKAGE_DIRECTORIES-}" ]; then '
            "printf 'ROSLISP_SET=yes\\n'; else printf 'ROSLISP_SET=no\\n'; fi"
        )
        lines = self.run_shell(prelude + printf_cpp + printf_lisp)
        self.assertEqual(
            self.marked(lines, "CMAKE_PREFIX_PATH"),
            CANONICAL_WORKSPACE + "/devel:/opt/ros/noetic",
        )
        self.assertEqual(self.marked(lines, "ROSLISP_SET"), "no")

    def test_missing_required_setup_fails_before_exporting_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            script = root / "workspace" / "src" / "air_ground_bringup" / "scripts" / SETUP.name
            script.parent.mkdir(parents=True)
            shutil.copy2(SETUP, script)
            (root / "PX4-Autopilot").mkdir()
            completed = subprocess.run(
                [
                    "bash",
                    "--noprofile",
                    "--norc",
                    "-c",
                    "source {}".format(shlex.quote(str(script))),
                ],
                env={
                    "HOME": os.environ["HOME"],
                    "PATH": os.environ["PATH"],
                    "SHELL": "/bin/bash",
                },
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("Missing required MVP setup path", completed.stderr)

    def test_forbidden_legacy_workspace_source_is_rejected_without_pollution(self):
        completed = subprocess.run(
            [
                "bash",
                "--noprofile",
                "--norc",
                "-c",
                "export ROS_PACKAGE_PATH=/opt/ros/noetic/share;"
                "source {0} >/dev/null;"
                "printf 'rc=%d\\n' \"$?\";"
                "printf 'RPP_AFTER=%s\\n' \"$ROS_PACKAGE_PATH\"".format(
                    shlex.quote(
                        FORBIDDEN_PATHS[0]
                        + "/"
                        + "/".join(SETUP.relative_to(PHYSICAL_WORKSPACE).parts)
                    )
                ),
            ],
            env={
                "HOME": os.environ["HOME"],
                "PATH": os.environ["PATH"],
                "SHELL": "/bin/bash",
            },
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        lines = completed.stdout.splitlines()
        self.assertEqual(self.marked(lines, "rc"), "1")
        self.assertEqual(
            self.marked(lines, "RPP_AFTER"), "/opt/ros/noetic/share"
        )
        self.assertIn("Forbidden legacy path", completed.stderr)

    def test_additional_workspace_alias_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            workspace_alias = root / "Ego_Planner_v2"
            px4_alias = root / "PX4-Autopilot"
            workspace_alias.symlink_to(PHYSICAL_WORKSPACE, target_is_directory=True)
            px4_alias.symlink_to(Path(CANONICAL_PX4), target_is_directory=True)
            alias_setup = workspace_alias / SETUP.relative_to(PHYSICAL_WORKSPACE)
            completed = subprocess.run(
                [
                    "bash",
                    "--noprofile",
                    "--norc",
                    "-c",
                    "source {} >/dev/null".format(shlex.quote(str(alias_setup))),
                ],
                env={
                    "HOME": os.environ["HOME"],
                    "PATH": os.environ["PATH"],
                    "SHELL": "/bin/bash",
                },
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            # A pre-fix wrapper can rewrite the shared marker through the alias.
            self.run_shell(self.source_command())

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("canonical workspace path", completed.stderr)


if __name__ == "__main__":
    unittest.main()
