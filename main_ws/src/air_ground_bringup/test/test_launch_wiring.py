#!/usr/bin/env python3

from pathlib import Path
import math
import os
import re
import unittest
import xml.etree.ElementTree as ET

import yaml


WORKSPACE = Path(__file__).parents[3]
BRINGUP = WORKSPACE / "src" / "air_ground_bringup"
COORDINATE = WORKSPACE / "src" / "air_ground_coordinate_transform"
EXPERIMENTS = WORKSPACE / "src" / "air_ground_experiments"
UGV_GAZEBO = WORKSPACE / "src" / "air_ground_ugv_gazebo"


def launch_tree(package, name):
    return ET.parse(package / "launch" / name).getroot()


def named_child(parent, tag, name):
    child = next(
        (child for child in parent.findall(tag) if child.get("name") == name),
        None,
    )
    if child is None:
        raise AssertionError("missing {} named {!r}".format(tag, name))
    return child


def included_launch(root, filename):
    return next(
        include
        for include in root.findall("include")
        if include.get("file", "").endswith("/launch/" + filename)
    )


def launch_tokens(argument_string):
    return re.findall(r"\$\([^)]*\)|\S+", argument_string)


def node_parameters(node):
    return {parameter.get("name"): parameter.get("value") for parameter in node.findall("param")}


def mission_ros_contract():
    mission_path = BRINGUP / "scripts" / "uav_sphere_mission.py"
    tree = __import__("ast").parse(mission_path.read_text(), filename=str(mission_path))
    defaults = {}
    subscribers = []
    publishers = []
    for call in (node for node in __import__("ast").walk(tree) if isinstance(node, __import__("ast").Call)):
        function = call.func
        if not isinstance(function, __import__("ast").Attribute):
            continue
        if function.attr == "get_param" and len(call.args) >= 2:
            if all(isinstance(argument, __import__("ast").Constant) for argument in call.args[:2]):
                defaults[call.args[0].value] = call.args[1].value
        if function.attr not in ("Subscriber", "Publisher") or not call.args:
            continue
        topic = call.args[0]
        if isinstance(topic, __import__("ast").Constant):
            destination = subscribers if function.attr == "Subscriber" else publishers
            destination.append(topic.value)
    return defaults, subscribers, publishers


class LaunchWiringTest(unittest.TestCase):
    def test_uav_yaw_reaches_spawn_model_as_gazebo_yaw(self):
        uav = launch_tree(BRINGUP, "uav_sitl.launch")
        yaw = named_child(uav, "arg", "yaw")
        spawn = named_child(uav.find("group"), "node", "iris_0_spawn")
        tokens = launch_tokens(spawn.get("args"))

        self.assertEqual(yaw.get("default"), "0.0")
        self.assertIn("-Y", tokens)
        self.assertEqual(tokens[tokens.index("-Y") + 1], "$(arg yaw)")

    def test_parent_launches_forward_independent_uav_and_ugv_yaw(self):
        mvp = launch_tree(BRINGUP, "mvp_system.launch")
        final_demo = launch_tree(BRINGUP, "air_ground_final_demo.launch")

        self.assertEqual(named_child(mvp, "arg", "uav_yaw").get("default"), "0.0")
        self.assertEqual(named_child(mvp, "arg", "ugv_yaw").get("default"), "0.0")
        uav_include = included_launch(mvp, "uav_sitl.launch")
        ugv_include = included_launch(mvp, "spawn_ugv.launch")
        for leaf_name, parent_name in (("x", "uav_x"), ("y", "uav_y"), ("yaw", "uav_yaw")):
            self.assertEqual(
                named_child(uav_include, "arg", leaf_name).get("value"),
                "$(arg {})".format(parent_name),
            )
        for leaf_name, parent_name in (("x", "ugv_x"), ("y", "ugv_y"), ("yaw", "ugv_yaw")):
            self.assertEqual(
                named_child(ugv_include, "arg", leaf_name).get("value"),
                "$(arg {})".format(parent_name),
            )

        self.assertEqual(named_child(final_demo, "arg", "uav_yaw").get("default"), "0.0")
        self.assertEqual(named_child(final_demo, "arg", "ugv_yaw").get("default"), "0.0")
        mvp_include = included_launch(final_demo, "mvp_system.launch")
        for name in ("uav_x", "uav_y", "uav_yaw", "ugv_x", "ugv_y", "ugv_yaw"):
            self.assertEqual(
                named_child(mvp_include, "arg", name).get("value"),
                "$(arg {})".format(name),
            )

    def test_registration_offsets_reach_the_mission_independently(self):
        final_demo = launch_tree(BRINGUP, "air_ground_final_demo.launch")
        self.assertEqual(named_child(final_demo, "arg", "registration_dx").get("default"), "1.6")
        self.assertEqual(named_child(final_demo, "arg", "registration_dy").get("default"), "0.0")

        mission = named_child(final_demo, "node", "uav_sphere_mission")
        self.assertEqual(
            named_child(mission, "param", "registration_dx").get("value"),
            "$(arg registration_dx)",
        )
        self.assertEqual(
            named_child(mission, "param", "registration_dy").get("value"),
            "$(arg registration_dy)",
        )

    def test_registration_altitude_is_a_research_launch_argument(self):
        experiment = launch_tree(BRINGUP, "air_ground_inspection_experiment.launch")
        altitude = named_child(experiment, "arg", "registration_altitude")

        self.assertEqual(altitude.get("default"), "1.8")
        mission = named_child(experiment, "node", "uav_sphere_mission")
        self.assertEqual(
            node_parameters(mission)["registration_altitude"],
            "$(arg registration_altitude)",
        )

    def test_reregistration_offsets_are_independent_launch_arguments(self):
        experiment = launch_tree(BRINGUP, "air_ground_inspection_experiment.launch")
        mission = named_child(experiment, "node", "uav_sphere_mission")
        parameters = node_parameters(mission)

        self.assertEqual(
            named_child(experiment, "arg", "reregistration_dx").get("default"),
            "-0.05",
        )
        self.assertEqual(
            named_child(experiment, "arg", "reregistration_dy").get("default"),
            "0.0",
        )
        self.assertEqual(
            parameters["reregistration_dx"], "$(arg reregistration_dx)"
        )
        self.assertEqual(
            parameters["reregistration_dy"], "$(arg reregistration_dy)"
        )

    def test_manual_acquisition_geometry_covers_the_charuco_pattern(self):
        """Derive the registration viewpoint contract from producer geometry.

        Pattern half-extents come from the texture generator, the mount comes
        from the UGV SDF/model frames and the detector config, camera optics
        come from the iris_D435i nadir camera, and the viewpoint identity comes
        from the research launch defaults.
        """
        generator = (UGV_GAZEBO / "tools" / "generate_charuco_texture.py").read_text()
        squares_x = int(re.search(r"SQUARES_X\s*=\s*(\d+)", generator).group(1))
        square_length = float(
            re.search(r"SQUARE_LENGTH_M\s*=\s*([0-9.]+)", generator).group(1)
        )
        half_pattern_x = squares_x * square_length / 2.0

        model = ET.parse(
            UGV_GAZEBO / "models" / "ugv_mvp" / "model.sdf"
        ).getroot()
        model_element = model.find("model")
        links = {link.get("name"): link for link in model_element.findall("link")}
        base_z = float(links["base_link"].find("pose").text.split()[2])
        board_pose = [float(v) for v in links["charuco_board"].find("pose").text.split()[:3]]
        board_center_rel_base = (
            board_pose[0],
            board_pose[1],
            round(board_pose[2] - base_z, 9),
        )
        # Pattern plane top face sits on the backing box top; compute the
        # OpenCV min-corner mount used by registration.yaml instead.
        backing = next(
            visual
            for visual in links["charuco_board"].findall("visual")
            if visual.get("name") == "backing"
        )
        backing_size = [
            float(v)
            for v in backing.find("geometry").find("box").find("size").text.split()
        ]

        with (COORDINATE / "config" / "registration.yaml").open() as stream:
            registration = yaml.safe_load(stream)
        board_mount = registration["ugv_base_to_board_translation"]
        camera_mount_z = float(registration["uav_base_to_camera_translation"][2])
        minimum_height = float(registration["minimum_uav_height"])

        experiment = launch_tree(BRINGUP, "air_ground_inspection_experiment.launch")

        def arg_default(name):
            return float(named_child(experiment, "arg", name).get("default"))

        uav_x, ugv_x = arg_default("uav_x"), arg_default("ugv_x")
        dx, dy = arg_default("registration_dx"), arg_default("registration_dy")
        altitude = arg_default("registration_altitude")

        separation = ugv_x - uav_x
        # Iris rotor envelope (~0.32 m) + UGV chassis half-length (~0.36 m)
        # plus a >=0.30 m takeoff gap.
        self.assertGreaterEqual(separation, 0.98)
        # Hover nadir lands on the ChArUco pattern centre, not the UGV origin.
        self.assertAlmostEqual(
            dx,
            separation - (-board_center_rel_base[0]),
            places=9,
        )
        self.assertEqual(dy, 0.0)

        # Nadir camera geometry from the actual SITL model.
        ugv_root = os.environ.get("UAV_UGV_ROOT", str(Path.home() / "UAV-UGV_ws"))
        iris_sdf_path = Path(
            ugv_root, "PX4-Autopilot/Tools/sitl_gazebo/models/iris_D435i/iris_D435i.sdf"
        )
        self.assertTrue(iris_sdf_path.is_file())
        iris = ET.parse(str(iris_sdf_path)).getroot()
        down_link = next(
            link for link in iris.iter("link") if link.get("name") == "down_cam_link"
        )
        down_pose_z = float(down_link.find("pose").text.split()[2])
        camera_element = down_link.find(".//camera")
        horizontal_fov = float(camera_element.find("horizontal_fov").text)
        width = int(camera_element.find("image").find("width").text)
        height = int(camera_element.find("image").find("height").text)
        self.assertAlmostEqual(down_pose_z, camera_mount_z, places=6)

        tan_vertical = math.tan(horizontal_fov / 2.0) * height / width
        # Bias budget: EKF home/yaw capture plus hover tracking error.
        bias_budget = 0.10
        edge_reserve = 0.15
        # Conservative convention: treat the commanded local-z as world height
        # above ground so the requirement can only get easier in reality.
        pattern_plane_world = board_pose[2]
        clearance_required = (
            bias_budget + half_pattern_x + edge_reserve
        ) / tan_vertical
        altitude_required = (
            clearance_required - down_pose_z + pattern_plane_world
        )
        self.assertGreaterEqual(
            altitude,
            altitude_required,
            "registration altitude does not clear the pattern with reserve",
        )
        self.assertGreaterEqual(
            altitude,
            minimum_height + 0.4,
            "altitude must exceed the observation height gate with margin",
        )

        # registration.yaml mount must equal SDF-derived OpenCV corner origin.
        half_pattern_y = (
            int(re.search(r"SQUARES_Y\s*=\s*(\d+)", generator).group(1))
            * square_length
            / 2.0
        )
        self.assertAlmostEqual(
            board_mount[0], board_center_rel_base[0] - half_pattern_x, places=6
        )
        self.assertAlmostEqual(
            board_mount[1], board_center_rel_base[1] - half_pattern_y, places=6
        )
        self.assertAlmostEqual(board_mount[2], board_center_rel_base[2], places=6)

    def test_legacy_final_demo_keeps_the_script_registration_altitude_default(self):
        final_demo = launch_tree(BRINGUP, "air_ground_final_demo.launch")
        mission = named_child(final_demo, "node", "uav_sphere_mission")

        self.assertNotIn("registration_altitude", node_parameters(mission))

    def test_research_default_uses_visual_yaw_while_legacy_demo_is_fixed_yaw(self):
        with (COORDINATE / "config" / "registration.yaml").open() as stream:
            registration = yaml.safe_load(stream)
        coordinate = launch_tree(COORDINATE, "coordinate_transform.launch")
        mvp = launch_tree(BRINGUP, "mvp_system.launch")
        final_demo = launch_tree(BRINGUP, "air_ground_final_demo.launch")

        self.assertIs(registration["use_visual_frame_yaw"], True)
        self.assertEqual(registration["uav_odom_input_parent_frame"], "map")
        self.assertEqual(registration["uav_odom_input_child_frame"], "base_link")
        self.assertEqual(registration["ugv_odom_input_parent_frame"], "ugv_0/odom")
        self.assertEqual(registration["ugv_odom_input_child_frame"], "ugv_0/base_link")
        self.assertEqual(
            registration["observation_input_frame"],
            "iris_0/nadir_camera_optical_frame",
        )
        self.assertEqual(
            named_child(coordinate, "arg", "use_visual_frame_yaw").get("default"),
            "true",
        )
        coordinate_node = named_child(coordinate, "node", "takeoff_registration")
        self.assertEqual(
            named_child(coordinate_node, "param", "use_visual_frame_yaw").get("value"),
            "$(arg use_visual_frame_yaw)",
        )
        self.assertEqual(named_child(mvp, "arg", "use_visual_frame_yaw").get("default"), "true")
        coordinate_include = included_launch(mvp, "coordinate_transform.launch")
        self.assertEqual(
            named_child(coordinate_include, "arg", "use_visual_frame_yaw").get("value"),
            "$(arg use_visual_frame_yaw)",
        )
        mvp_include = included_launch(final_demo, "mvp_system.launch")
        self.assertEqual(
            named_child(mvp_include, "arg", "use_visual_frame_yaw").get("value"),
            "false",
        )

    def test_registration_mode_and_repeated_window_parameters_are_wired(self):
        with (COORDINATE / "config" / "registration.yaml").open() as stream:
            registration = yaml.safe_load(stream)
        coordinate = launch_tree(COORDINATE, "coordinate_transform.launch")
        node = named_child(coordinate, "node", "takeoff_registration")

        self.assertEqual(registration.get("registration_mode"), "one_shot")
        self.assertGreater(registration["registration_window_seconds"], 0.0)
        self.assertGreaterEqual(
            registration["registration_window_max_samples"],
            registration["minimum_samples"],
        )
        self.assertGreater(registration["periodic_update_seconds"], 0.0)
        self.assertGreater(
            registration["degraded_covariance_trace_threshold"], 0.0
        )
        self.assertEqual(
            named_child(coordinate, "arg", "registration_mode").get("default"),
            "one_shot",
        )
        self.assertEqual(
            named_child(node, "param", "registration_mode").get("value"),
            "$(arg registration_mode)",
        )

    def test_research_registration_frames_follow_actual_perturbation_producers(self):
        perturbation = launch_tree(EXPERIMENTS, "frame_perturbation.launch")
        research = launch_tree(BRINGUP, "air_ground_inspection_experiment.launch")
        uav_producer = named_child(perturbation, "node", "uav_odom_perturbation")
        ugv_producer = named_child(perturbation, "node", "ugv_odom_perturbation")
        uav_parameters = node_parameters(uav_producer)
        ugv_parameters = node_parameters(ugv_producer)

        self.assertEqual(uav_parameters["twist_convention"], "parent")
        uav_parent = uav_parameters["destination_frame"]
        uav_child = uav_parent
        self.assertEqual(ugv_parameters["twist_convention"], "body")
        ugv_parent = ugv_parameters["destination_frame"]

        spawn_include = included_launch(research, "spawn_ugv.launch")
        self.assertIsNotNone(spawn_include)
        spawn = launch_tree(UGV_GAZEBO, "spawn_ugv.launch")
        spawn_node = named_child(spawn, "node", "spawn_$(arg name)")
        model_match = re.search(r"models/([^\s]+)/model\.sdf", spawn_node.get("args"))
        self.assertIsNotNone(model_match)
        model = ET.parse(
            UGV_GAZEBO / "models" / model_match.group(1) / "model.sdf"
        ).getroot()
        planar_plugin = next(
            plugin
            for plugin in model.iter("plugin")
            if plugin.get("filename") == "libgazebo_ros_planar_move.so"
        )
        ugv_child = planar_plugin.find("robotBaseFrame").text

        registration = named_child(research, "node", "takeoff_registration")
        registration_parameters = node_parameters(registration)
        mission = named_child(research, "node", "uav_sphere_mission")
        mission_parameters = node_parameters(mission)
        monitor = named_child(research, "node", "ugv_coordinate_monitor")
        monitor_parameters = node_parameters(monitor)
        controller = named_child(research, "node", "ugv_goal_controller")
        controller_parameters = node_parameters(controller)

        self.assertEqual(
            registration_parameters.get("uav_odom_input_parent_frame"), uav_parent
        )
        self.assertEqual(
            registration_parameters.get("uav_odom_input_child_frame"), uav_child
        )
        self.assertEqual(
            registration_parameters.get("ugv_odom_input_parent_frame"), ugv_parent
        )
        self.assertEqual(
            registration_parameters.get("ugv_odom_input_child_frame"), ugv_child
        )
        self.assertEqual(registration_parameters.get("uav_odom_frame"), uav_parent)
        self.assertEqual(registration_parameters.get("ugv_odom_frame"), ugv_parent)
        self.assertEqual(mission_parameters.get("uav_odom_frame"), uav_parent)
        self.assertEqual(mission_parameters.get("ugv_odom_frame"), ugv_parent)
        self.assertEqual(
            monitor_parameters.get("odom_topic"),
            ugv_parameters["destination_topic"],
        )
        self.assertEqual(
            controller_parameters.get("odom_topic"),
            ugv_parameters["destination_topic"],
        )
        self.assertEqual(controller_parameters.get("odom_frame"), ugv_parent)

        for node in perturbation.findall("node"):
            self.assertFalse(
                node.get("pkg") == "tf2_ros"
                and node.get("type") == "static_transform_publisher",
                "experiment perturbation must not expose injected transforms through TF",
            )
        perturbation_node_path = EXPERIMENTS / "scripts" / "odom_perturbation_node.py"
        perturbation_node_tree = __import__("ast").parse(
            perturbation_node_path.read_text(), filename=str(perturbation_node_path)
        )
        broadcaster_symbols = {
            node.attr
            for node in __import__("ast").walk(perturbation_node_tree)
            if isinstance(node, __import__("ast").Attribute)
        }
        self.assertNotIn("TransformBroadcaster", broadcaster_symbols)
        self.assertNotIn("StaticTransformBroadcaster", broadcaster_symbols)

    def test_uncertainty_handoff_is_research_opt_in_and_final_demo_opt_out(self):
        research = launch_tree(BRINGUP, "air_ground_inspection_experiment.launch")
        expected = {
            "uncertainty_aware_handoff": "false",
            "inspection_radius": "0.35",
            "inspection_yaw": "0.03490658503988659",
            "target_sigma_floor": "0.02",
            "reregistration_timeout": "60.0",
        }
        mission = named_child(research, "node", "uav_sphere_mission")
        parameters = node_parameters(mission)
        for name, default in expected.items():
            with self.subTest(name=name):
                self.assertEqual(named_child(research, "arg", name).get("default"), default)
                self.assertEqual(parameters.get(name), "$(arg {})".format(name))

        final_demo = launch_tree(BRINGUP, "air_ground_final_demo.launch")
        final_mission = named_child(final_demo, "node", "uav_sphere_mission")
        self.assertEqual(
            node_parameters(final_mission).get("uncertainty_aware_handoff"),
            "false",
        )

    def test_mission_wires_distinct_registration_channels_and_safe_defaults(self):
        defaults, subscribers, publishers = mission_ros_contract()

        self.assertEqual(defaults.get("~uncertainty_aware_handoff"), False)
        self.assertEqual(defaults.get("~inspection_radius"), 0.35)
        self.assertEqual(defaults.get("~inspection_yaw"), 0.03490658503988659)
        self.assertEqual(defaults.get("~target_sigma_floor"), 0.02)
        self.assertEqual(defaults.get("~reregistration_timeout"), 60.0)
        self.assertIn("/air_ground/registration/accepted_update", subscribers)
        self.assertIn("/air_ground/registration/estimate", subscribers)
        self.assertIn("/air_ground/handoff/action", publishers)
        self.assertIn("/air_ground/handoff/confidence_radius", publishers)
        for topic in subscribers:
            self.assertNotIn("gazebo", topic.lower())
            self.assertNotIn("truth", topic.lower())
            self.assertNotIn("model_states", topic.lower())

    def test_task8_python_package_and_focused_tests_are_registered(self):
        setup_path = BRINGUP / "setup.py"
        self.assertTrue(setup_path.is_file(), "air_ground_bringup setup.py is missing")
        setup_tree = __import__("ast").parse(setup_path.read_text(), filename=str(setup_path))
        package_literals = [
            node.value
            for node in __import__("ast").walk(setup_tree)
            if isinstance(node, __import__("ast").Constant) and isinstance(node.value, str)
        ]
        self.assertIn("air_ground_bringup", package_literals)
        self.assertIn("src", package_literals)

        cmake = (BRINGUP / "CMakeLists.txt").read_text()
        self.assertIn("catkin_python_setup()", cmake)
        self.assertIn("catkin_add_nosetests(test/test_target_handoff.py)", cmake)
        self.assertIn("catkin_add_nosetests(test/test_reregistration_state_machine.py)", cmake)
        self.assertIn("catkin_add_nosetests(test/test_setup_mvp_env.py)", cmake)
        self.assertIn("catkin_add_nosetests(test/test_ugv_goal_controller.py)", cmake)
        self.assertIn("catkin_add_nosetests(test/test_launch_wiring.py)", cmake)
        self.assertIn("catkin_add_nosetests(test/test_auto_takeoff_compat.py)", cmake)
        self.assertIn("catkin_add_nosetests(test/test_offboard_compat.py)", cmake)

        package = ET.parse(BRINGUP / "package.xml").getroot()
        runtime_dependencies = [node.text for node in package.findall("exec_depend")]
        self.assertIn("python3-numpy", runtime_dependencies)

    def test_direct_tf_import_has_catkin_and_manifest_dependency(self):
        mission_path = BRINGUP / "scripts" / "uav_sphere_mission.py"
        mission_tree = __import__("ast").parse(
            mission_path.read_text(), filename=str(mission_path)
        )
        imported_modules = {
            node.module
            for node in mission_tree.body
            if isinstance(node, __import__("ast").ImportFrom)
        }
        self.assertIn("tf.transformations", imported_modules)

        cmake = (BRINGUP / "CMakeLists.txt").read_text()
        find_components = re.search(
            r"find_package\(catkin REQUIRED COMPONENTS(?P<body>.*?)\)",
            cmake,
            re.DOTALL,
        ).group("body").split()
        exported_components = re.search(
            r"catkin_package\(CATKIN_DEPENDS(?P<body>.*?)\)",
            cmake,
            re.DOTALL,
        ).group("body").split()
        self.assertIn("tf", find_components)
        self.assertIn("tf", exported_components)

        package = ET.parse(BRINGUP / "package.xml").getroot()
        direct_dependencies = [node.text for node in package.findall("depend")]
        self.assertIn("tf", direct_dependencies)


if __name__ == "__main__":
    unittest.main()
