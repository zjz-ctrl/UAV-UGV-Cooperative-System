#!/usr/bin/env python3

import sys
from pathlib import Path
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from air_ground_experiments.m2c_validation import (  # noqa: E402
    FAIL,
    INCONCLUSIVE,
    PASS,
    accepted_events,
    action_events,
    estimate_events,
    grade_a,
    grade_b,
    grade_c,
    grade_d,
    parse_echo_csv,
    phase_events,
    scalar_events,
)
from run_m2c_dynamic_validation import load_control_times  # noqa: E402


def accepted(stamp, revision, cxx=0.0001, cyy=0.0001, cyaw=0.00003):
    return (stamp, revision, cxx, cyy, cyaw)


class ParsingTest(unittest.TestCase):
    def test_echo_nanoseconds_are_normalized_for_early_and_late_rows(self):
        rows = parse_echo_csv(
            "%time,field.data\n"
            "146000000,CAPTURING_ORIGIN\n"
            "12000000000,PRESTREAM\n"
        )

        self.assertEqual(
            phase_events(rows),
            [(0.146, "CAPTURING_ORIGIN"), (12.0, "PRESTREAM")],
        )

    def test_phase_and_scalar_csv_round_trip(self):
        rows = parse_echo_csv("%time,field.data\n12.0,PRESTREAM\n15.0,OFFBOARD\n")
        self.assertEqual(
            phase_events(rows), [(12.0, "PRESTREAM"), (15.0, "OFFBOARD")]
        )

    def test_accepted_and_estimate_rows_expose_covariance_slots(self):
        accepted_rows = parse_echo_csv(
            "%time,field.revision,field.pose.covariance0,"
            "field.pose.covariance7,field.pose.covariance35\n"
            "33.7,1,0.0001,0.0002,0.00003\n"
        )
        self.assertEqual(
            accepted_events(accepted_rows), [(33.7, 1, 0.0001, 0.0002, 0.00003)]
        )
        estimate_rows = parse_echo_csv(
            "%time,field.pose.covariance0,field.pose.covariance7,"
            "field.pose.covariance35\n81.7,0.03,0.04,0.002\n"
        )
        self.assertEqual(
            estimate_events(estimate_rows), [(81.7, 0.03, 0.04, 0.002)]
        )
        revision_rows = parse_echo_csv("%time,field.data\n81.7,2\n")
        self.assertEqual(scalar_events(revision_rows)[0][1], 2.0)

    def test_control_event_nanoseconds_are_normalized_to_seconds(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "control_events.csv"
            path.write_text(
                "command,sim_time\n"
                "hide,27734000000.000000\n"
                "outlier 1.5 0.8 0.5,158234000000.000000\n"
            )

            controls = load_control_times(Path(directory))

        self.assertEqual(controls["hide"], 27.734)
        self.assertEqual(controls["outlier"], 158.234)


class GradeATest(unittest.TestCase):
    def direct_run(self):
        return dict(
            phases=[(10.0, "PRESTREAM"), (81.9, "DISPATCH"), (82.2, "OVERWATCH")],
            actions=[(81.7, "DIRECT")],
            confidence=[(81.7, 0.29)],
            inliers=[(33.7, 20)],
            accepted=[accepted(33.7, 1)],
            estimates=[(33.8, 0.0001, 0.0001, 0.00002), (81.7, 0.02, 0.02, 0.0002)],
            goals=[(82.0, 1)],
        )

    def test_low_uncertainty_direct_chain_passes(self):
        verdict = grade_a(budget_radius=0.35, budget_yaw=0.0349, **self.direct_run())

        self.assertEqual(verdict.status, PASS)
        self.assertEqual(verdict.evidence["revision1"], 1)
        self.assertEqual(verdict.evidence["DIRECT"], 1)
        self.assertEqual(verdict.evidence["GOAL"], 1)

    def test_reregister_instead_of_direct_is_inconclusive_input(self):
        run = self.direct_run()
        run["actions"] = [(81.7, "REREGISTER")]
        run["confidence"] = [(81.7, 0.4556)]
        run["estimates"] = [(81.7, 0.0343, 0.0343, 0.00204)]

        verdict = grade_a(budget_radius=0.35, budget_yaw=0.0349, **run)

        self.assertEqual(verdict.status, INCONCLUSIVE)
        self.assertAlmostEqual(verdict.evidence["confidence"], 0.4556)
        self.assertGreater(verdict.evidence["sigma_yaw_deg"], 2.0)

    def test_forbidden_reregistration_phase_fails_direct_run(self):
        run = self.direct_run()
        run["phases"] = run["phases"] + [(82.0, "RETURN_TO_UGV")]

        verdict = grade_a(budget_radius=0.35, budget_yaw=0.0349, **run)

        self.assertEqual(verdict.status, FAIL)

    def test_confidence_above_budget_fails_even_with_direct(self):
        run = self.direct_run()
        run["confidence"] = [(81.7, 0.5)]

        verdict = grade_a(budget_radius=0.35, budget_yaw=0.0349, **run)

        self.assertEqual(verdict.status, FAIL)

    def test_direct_without_confidence_fails_with_evidence(self):
        run = self.direct_run()
        run["confidence"] = []

        try:
            verdict = grade_a(budget_radius=0.35, budget_yaw=0.0349, **run)
        except IndexError:
            self.fail("grade_a raised IndexError for missing confidence")

        self.assertEqual(verdict.status, FAIL)
        self.assertIsNone(verdict.evidence["confidence"])
        self.assertIn("confidence radius not published", verdict.reasons)

    def test_missing_goal_or_overwatch_fails(self):
        for drop in ("goals",):
            run = self.direct_run()
            run[drop] = []
            verdict = grade_a(budget_radius=0.35, budget_yaw=0.0349, **run)
            self.assertEqual(verdict.status, FAIL)
        run = self.direct_run()
        run["phases"] = [phase for phase in run["phases"] if phase[1] != "OVERWATCH"]
        verdict = grade_a(budget_radius=0.35, budget_yaw=0.0349, **run)
        self.assertEqual(verdict.status, FAIL)


class GradeBTest(unittest.TestCase):
    def reregister_run(self):
        return dict(
            phases=[(81.7, "RETURN_TO_UGV"), (104.6, "WAIT_REREGISTRATION")],
            actions=[(81.7, "REREGISTER")],
            accepted=[accepted(33.7, 1)],
            revision_values=[(34.0, 1.0), (110.0, 1.0)],
            estimates=[(33.8, 0.0001, 0.0001, 0.00002), (81.7, 0.0343, 0.0343, 0.002)],
            goals=[],
            observation_dest_times=[30.0, 33.0, 33.9],
            hide_time=34.0,
        )

    def test_hidden_observations_keep_revision_at_one_and_pass(self):
        verdict = grade_b(**self.reregister_run())

        self.assertEqual(verdict.status, PASS)
        self.assertEqual(verdict.evidence["max_revision"], 1)
        self.assertEqual(verdict.evidence["WAIT_REREGISTRATION"], 1)

    def test_revision_advance_fails(self):
        run = self.reregister_run()
        run["accepted"] = run["accepted"] + [accepted(90.0, 2)]
        run["revision_values"] = [(34.0, 1.0), (110.0, 2.0)]

        self.assertEqual(grade_b(**run).status, FAIL)

    def test_observations_still_flowing_after_hide_fails(self):
        run = self.reregister_run()
        run["observation_dest_times"] = [30.0, 40.0]

        self.assertEqual(grade_b(**run).status, FAIL)

    def test_dispatch_after_reregister_fails(self):
        run = self.reregister_run()
        run["phases"] = run["phases"] + [(110.0, "DISPATCH")]

        self.assertEqual(grade_b(**run).status, FAIL)


class GradeCTest(unittest.TestCase):
    def resume_run(self):
        return dict(
            phases=[
                (81.7, "RETURN_TO_UGV"),
                (104.6, "WAIT_REREGISTRATION"),
                (107.5, "RESUME_HANDOFF"),
                (107.53, "DISPATCH"),
                (107.57, "OVERWATCH"),
            ],
            actions=[(81.7, "REREGISTER")],
            accepted=[accepted(33.7, 1), accepted(107.48, 2)],
            revision_values=[(34.0, 1.0), (110.0, 2.0)],
            estimates=[
                (33.8, 0.0001, 0.0001, 0.00002),
                (104.5, 0.0574, 0.0574, 0.0034),
                (107.6, 0.00012, 0.00018, 0.00004),
            ],
            goals=[(107.569, 1)],
        )

    def test_clean_revision_two_resume_passes(self):
        verdict = grade_c(**self.resume_run())

        self.assertEqual(verdict.status, PASS)
        self.assertGreater(verdict.evidence["cov_before"], verdict.evidence["cov_after"])
        self.assertLess(
            verdict.evidence["sigma_yaw_after"], verdict.evidence["sigma_yaw_before"]
        )

    def test_missing_revision_two_fails(self):
        run = self.resume_run()
        run["accepted"] = run["accepted"][:1]

        self.assertEqual(grade_c(**run).status, FAIL)

    def test_double_revision_event_fails(self):
        run = self.resume_run()
        run["accepted"] = run["accepted"] + [accepted(107.6, 2)]

        self.assertEqual(grade_c(**run).status, FAIL)

    def test_goal_before_dispatch_fails(self):
        run = self.resume_run()
        run["goals"] = [(100.0, 1)]

        self.assertEqual(grade_c(**run).status, FAIL)

    def test_resume_before_revision_two_fails(self):
        run = self.resume_run()
        run["phases"] = [
            (81.7, "RETURN_TO_UGV"),
            (90.0, "WAIT_REREGISTRATION"),
            (95.0, "RESUME_HANDOFF"),
            (95.1, "DISPATCH"),
        ]
        run["accepted"] = [accepted(33.7, 1), accepted(107.48, 2)]

        self.assertEqual(grade_c(**run).status, FAIL)


class GradeDTest(unittest.TestCase):
    def outlier_run(self):
        return dict(
            phases=[(81.7, "RETURN_TO_UGV"), (104.6, "WAIT_REREGISTRATION")],
            actions=[(81.7, "REREGISTER")],
            accepted=[accepted(33.7, 1)],
            revision_values=[(34.0, 1.0), (115.0, 1.0)],
            estimates=[
                (104.5, 0.0574, 0.0574, 0.0034),
                (108.0, 0.0576, 0.0576, 0.0035),
            ],
            goals=[],
            statuses=[(109.5, "REJECTED"), (112.0, "REJECTED")],
            innovations=[(109.5, 4321.0), (112.0, 5123.0)],
            observation_dest_times=[30.0, 33.9],
            hide_time=34.0,
            outlier_time=107.0,
        )

    def test_outlier_rejected_revision_stays_one_passes(self):
        verdict = grade_d(**self.outlier_run())

        self.assertEqual(verdict.status, PASS)
        self.assertEqual(verdict.evidence["revision_after"], 1)
        self.assertGreater(verdict.evidence["NIS"], 11.344866730144373)
        self.assertEqual(verdict.evidence["RESUME_HANDOFF"], 0)

    def test_revision_two_after_outlier_fails(self):
        run = self.outlier_run()
        run["accepted"] = run["accepted"] + [accepted(112.0, 2)]
        run["revision_values"] = [(34.0, 1.0), (115.0, 2.0)]

        self.assertEqual(grade_d(**run).status, FAIL)

    def test_no_rejection_evidence_fails(self):
        run = self.outlier_run()
        run["statuses"] = [(109.5, "TRACKING")]
        run["innovations"] = []

        self.assertEqual(grade_d(**run).status, FAIL)

    def test_resume_after_outlier_fails(self):
        run = self.outlier_run()
        run["phases"] = run["phases"] + [(110.0, "RESUME_HANDOFF")]

        self.assertEqual(grade_d(**run).status, FAIL)

    def test_covariance_false_improvement_fails(self):
        run = self.outlier_run()
        run["estimates"] = [
            (104.5, 0.0574, 0.0574, 0.0034),
            (108.0, 0.0001, 0.0001, 0.00002),
        ]

        self.assertEqual(grade_d(**run).status, FAIL)


if __name__ == "__main__":
    unittest.main()
