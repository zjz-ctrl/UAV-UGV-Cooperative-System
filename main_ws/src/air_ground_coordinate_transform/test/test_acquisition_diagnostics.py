#!/usr/bin/env python3

import ast
from pathlib import Path
import unittest

from air_ground_coordinate_transform.acquisition_diagnostics import (
    AcquisitionDiagnostics,
    KNOWN_REASONS,
    REASON_BELOW_HEIGHT,
    REASON_INTERPOLATION,
    REASON_NO_ODOM,
    REASON_ODOM_BRACKET,
)

NODE_PATH = (
    Path(__file__).parents[1] / "scripts" / "takeoff_registration.py"
)


class AcquisitionDiagnosticsTest(unittest.TestCase):
    def test_summary_reports_counts_in_a_stable_order(self):
        diagnostics = AcquisitionDiagnostics()
        for _ in range(6):
            diagnostics.observe()
        diagnostics.drop(REASON_NO_ODOM)
        diagnostics.drop(REASON_ODOM_BRACKET)
        diagnostics.drop(REASON_ODOM_BRACKET)
        diagnostics.accept()

        summary = diagnostics.summary()

        self.assertIn("received=6", summary)
        self.assertIn("dropped=3", summary)
        self.assertIn("no_odom=1", summary)
        self.assertIn("odom_bracket=2", summary)
        self.assertIn("sampled=1", summary)
        self.assertIn("last_drop=odom_bracket", summary)

    def test_idle_summary_explains_zero_observations(self):
        diagnostics = AcquisitionDiagnostics()

        summary = diagnostics.summary()

        self.assertIn("received=0", summary)
        self.assertIn("sampled=0", summary)

    def test_should_report_is_throttled_and_gated_on_activity(self):
        diagnostics = AcquisitionDiagnostics(throttle_seconds=5.0)

        self.assertFalse(diagnostics.should_report(10.0))

        diagnostics.observe()
        self.assertTrue(diagnostics.should_report(10.0))
        self.assertFalse(diagnostics.should_report(12.0))
        self.assertFalse(diagnostics.should_report(14.9))
        self.assertTrue(diagnostics.should_report(15.0))

    def test_unknown_reasons_are_rejected(self):
        diagnostics = AcquisitionDiagnostics()
        with self.assertRaises(ValueError):
            diagnostics.drop("mystery")

    def test_known_reason_table_is_complete_for_the_node_paths(self):
        for reason in (
            REASON_NO_ODOM,
            REASON_BELOW_HEIGHT,
            REASON_ODOM_BRACKET,
            REASON_INTERPOLATION,
        ):
            self.assertIn(reason, KNOWN_REASONS)

    def test_registration_node_reports_every_gate_drop_reason(self):
        source = NODE_PATH.read_text()
        self.assertIn("AcquisitionDiagnostics", source)

        import air_ground_coordinate_transform.acquisition_diagnostics as module

        drops = set()
        for node in ast.walk(ast.parse(source)):
            if not (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "drop"
                and node.args
            ):
                continue
            argument = node.args[0]
            if isinstance(argument, ast.Constant):
                drops.add(str(argument.value))
            elif isinstance(argument, ast.Name):
                drops.add(getattr(module, argument.id))
        expected = {"no_odom", "odom_bracket", "stamp_zero", "below_height",
                    "uav_fast", "uav_yaw_fast", "ugv_fast", "interpolation"}
        self.assertTrue(
            expected.issubset(drops),
            "takeoff_registration must report every silent drop reason, got {}".format(
                sorted(drops)
            ),
        )


if __name__ == "__main__":
    unittest.main()
