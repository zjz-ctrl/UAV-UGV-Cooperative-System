#!/usr/bin/env python3

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).parents[1]
SCRIPTS = PACKAGE / "scripts"



def _literal_or_constant(call_argument, tree):
    """Resolve an AST argument that is a literal or a module constant."""
    import ast as ast_module
    if isinstance(call_argument, ast_module.Constant):
        return call_argument.value
    if isinstance(call_argument, ast_module.Name):
        for node in tree.body:
            if (
                isinstance(node, ast_module.Assign)
                and len(node.targets) == 1
                and isinstance(node.targets[0], ast_module.Name)
                and node.targets[0].id == call_argument.id
            ):
                return ast_module.literal_eval(node.value)
    raise AssertionError("could not resolve topic expression")


class PackageSafetyTest(unittest.TestCase):
    def test_recorder_class_has_exactly_one_evaluation_status_publisher(self):
        tree = ast.parse((SCRIPTS / "experiment_recorder.py").read_text())
        recorder = next(
            node for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "ExperimentRecorder"
        )
        publishers = [
            node for node in ast.walk(recorder)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "Publisher"
        ]

        self.assertEqual(len(publishers), 1)
        self.assertEqual(
            _literal_or_constant(publishers[0].args[0], tree),
            "/air_ground_experiment/evaluation/status",
        )

    def test_gazebo_truth_is_confined_to_recorder(self):
        autonomy_scripts = [
            path for path in SCRIPTS.glob("*.py")
            if path.name != "experiment_recorder.py"
        ]
        for path in autonomy_scripts:
            source = path.read_text()
            self.assertNotIn("gazebo_msgs", source, path.name)
            self.assertNotIn("/gazebo/model_states", source, path.name)
        recorder_source = (SCRIPTS / "experiment_recorder.py").read_text()
        self.assertIn("/gazebo/model_states", recorder_source)

    def test_truth_topics_are_only_subscribed_by_the_evaluation_recorder(self):
        truth_subscribers = []
        for path in SCRIPTS.glob("*.py"):
            tree = ast.parse(path.read_text())
            for call in ast.walk(tree):
                if not (
                    isinstance(call, ast.Call)
                    and isinstance(call.func, ast.Attribute)
                    and call.func.attr == "Subscriber"
                    and call.args
                ):
                    continue
                try:
                    topic = _literal_or_constant(call.args[0], tree)
                except AssertionError:
                    continue
                if "/air_ground_experiment/truth/" in str(topic):
                    truth_subscribers.append(path.name)

        self.assertEqual(truth_subscribers, ["experiment_recorder.py", "experiment_recorder.py"])

    def test_launch_wires_both_odometry_outputs_and_all_adapters(self):
        root = ET.parse(PACKAGE / "launch" / "frame_perturbation.launch").getroot()
        nodes = {node.get("name"): node for node in root.findall("node")}

        self.assertEqual(
            {node.get("type") for node in nodes.values()},
            {
                "odom_perturbation_node.py",
                "observation_gate.py",
                "position_command_adapter.py",
                "experiment_recorder.py",
            },
        )
        self.assertEqual(
            nodes["uav_odom_perturbation"].find("param[@name='destination_topic']").get("value"),
            "/air_ground_experiment/uav/odom",
        )
        self.assertEqual(
            nodes["ugv_odom_perturbation"].find("param[@name='destination_topic']").get("value"),
            "/air_ground_experiment/ugv/odom",
        )
        self.assertEqual(
            nodes["observation_gate"].find("param[@name='destination_topic']").get("value"),
            "/air_ground_experiment/charuco/observation",
        )
        for name in ("uav_odom_perturbation", "ugv_odom_perturbation", "position_command_adapter"):
            initial = nodes[name].find("rosparam[@param='initial_xyyaw']")
            self.assertIsNotNone(initial)
            self.assertEqual(initial.get("subst_value"), "true")

    def test_package_installs_scripts_launch_and_registers_pure_tests(self):
        cmake = (PACKAGE / "CMakeLists.txt").read_text()
        package_xml = ET.parse(PACKAGE / "package.xml").getroot()
        dependencies = {
            element.text for element in package_xml
            if element.tag.endswith("depend")
        }

        for script in (
            "odom_perturbation_node.py",
            "observation_gate.py",
            "position_command_adapter.py",
            "experiment_recorder.py",
        ):
            self.assertIn("scripts/{}".format(script), cmake)
        self.assertIn("CATKIN_ENABLE_TESTING", cmake)
        self.assertIn("catkin_add_nosetests(test/test_frame_perturbation.py)", cmake)
        self.assertIn("catkin_add_nosetests(test/test_metrics.py)", cmake)
        self.assertIn("install(DIRECTORY launch", cmake)
        self.assertTrue(
            {"gazebo_msgs", "geometry_msgs", "nav_msgs", "quadrotor_msgs", "rospy", "std_msgs"}
            <= dependencies
        )

    def test_all_perturbation_consumers_share_one_epoch_parameter(self):
        root = ET.parse(PACKAGE / "launch" / "frame_perturbation.launch").getroot()
        nodes = {node.get("name"): node for node in root.findall("node")}
        self.assertIn("arg", [element.tag for element in root])
        epoch_arg = next(
            element for element in root.findall("arg") if element.get("name") == "epoch_seconds"
        )

        for name in (
            "uav_odom_perturbation",
            "ugv_odom_perturbation",
            "position_command_adapter",
            "observation_gate",
        ):
            param = nodes[name].find("param[@name='epoch_seconds']")
            self.assertIsNotNone(param, name)
            self.assertEqual(param.get("value"), "$(arg epoch_seconds)")

    def test_each_stream_uses_a_distinct_domain_label(self):
        root = ET.parse(PACKAGE / "launch" / "frame_perturbation.launch").getroot()
        nodes = {node.get("name"): node for node in root.findall("node")}
        expected = {
            "uav_odom_perturbation": "uav",
            "ugv_odom_perturbation": "ugv",
            "observation_gate": "gate",
            "position_command_adapter": "uav",
        }
        for name, domain in expected.items():
            param = nodes[name].find("param[@name='seed_domain']")
            self.assertIsNotNone(param, name)
            self.assertEqual(param.get("value"), domain)

    def test_twist_conventions_match_producer_ruling(self):
        root = ET.parse(PACKAGE / "launch" / "frame_perturbation.launch").getroot()
        nodes = {node.get("name"): node for node in root.findall("node")}
        expected = {
            "uav_odom_perturbation": "parent",
            "ugv_odom_perturbation": "body",
        }
        for name, convention in expected.items():
            param = nodes[name].find("param[@name='twist_convention']")
            self.assertIsNotNone(param, name)
            self.assertEqual(param.get("value"), convention)

    def test_recorder_launch_names_real_mission_and_anomaly_truth(self):
        root = ET.parse(PACKAGE / "launch" / "frame_perturbation.launch").getroot()
        node = next(
            child for child in root.findall("node")
            if child.get("name") == "experiment_recorder"
        )
        mission = node.find("param[@name='mission_phase_topic']")
        anomaly = node.find("param[@name='anomaly_model']")

        self.assertIsNotNone(mission)
        self.assertEqual(mission.get("value"), "/air_ground/mission_phase")
        self.assertIsNotNone(anomaly)
        self.assertEqual(anomaly.get("value"), "red_sphere")

    def test_observation_gate_control_topic_is_arg_wired_and_default_disabled(self):
        root = ET.parse(PACKAGE / "launch" / "frame_perturbation.launch").getroot()
        gate = next(
            child for child in root.findall("node")
            if child.get("name") == "observation_gate"
        )
        control = gate.find("param[@name='control_topic']")

        self.assertIsNotNone(control)
        self.assertEqual(control.get("value"), "$(arg observation_control_topic)")
        arg = next(
            element for element in root.findall("arg")
            if element.get("name") == "observation_control_topic"
        )
        self.assertEqual(arg.get("default"), "")

    def test_experiment_launch_declares_and_forwards_observation_control_topic(self):
        experiment = (
            Path(__file__).parents[2]
            / "air_ground_bringup"
            / "launch"
            / "air_ground_inspection_experiment.launch"
        )
        root = ET.parse(experiment).getroot()
        arg = next(
            element for element in root.findall("arg")
            if element.get("name") == "observation_control_topic"
        )
        self.assertEqual(arg.get("default"), "")
        include = next(
            child for child in root.findall("include")
            if child.get("file", "").endswith("frame_perturbation.launch")
        )
        forwarded = include.find("arg[@name='observation_control_topic']")
        self.assertIsNotNone(forwarded)
        self.assertEqual(forwarded.get("value"), "$(arg observation_control_topic)")


if __name__ == "__main__":
    unittest.main()
