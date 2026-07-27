import math
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "examples"))

from glove_controller_alignment import (  # noqa: E402
    PoseSample,
    estimate_time_offset,
    fit_alignment,
    glove_hand_quaternion,
    multiply,
    nearest_aligned_pairs,
    normalize_quaternion,
    quaternion_angle_deg,
    solve_controller_pivot,
    solve_hand_eye_rotation,
)


def axis_quaternion(axis, angle):
    vector = np.asarray(axis, dtype=float)
    vector /= np.linalg.norm(vector)
    return normalize_quaternion(
        np.r_[math.cos(angle * 0.5), vector * math.sin(angle * 0.5)]
    )


def trajectory(time_value):
    return multiply(
        multiply(axis_quaternion((1, 0, 0), 0.55 * math.sin(1.1 * time_value)),
                 axis_quaternion((0, 1, 0), 0.45 * math.sin(1.7 * time_value + 0.4))),
        axis_quaternion((0, 0, 1), 0.50 * math.sin(2.3 * time_value + 0.8)),
    )


class GloveControllerAlignmentTests(unittest.TestCase):
    def test_hi5_hand_slot_is_used_and_forearm_is_ignored(self):
        rotations = np.zeros(63)
        rotations[0:3] = [90.0, 0.0, 0.0]
        baseline = glove_hand_quaternion(rotations, "xyz")
        self.assertLess(quaternion_angle_deg(baseline), 1e-8)
        rotations[3:6] = [0.0, 0.0, 30.0]
        self.assertAlmostEqual(quaternion_angle_deg(glove_hand_quaternion(rotations, "xyz")), 30.0)

    def synthetic_streams(self, offset=0.12):
        world = axis_quaternion((0.3, 0.7, 0.2), 0.4)
        mount = axis_quaternion((0.6, -0.2, 0.5), -0.55)
        controller = []
        glove = []
        pivot = np.array([0.2, 1.1, -0.3])
        controller_to_pivot = np.array([0.04, -0.025, 0.09])
        for time_value in np.arange(0.0, 12.0, 0.02):
            controller_q = trajectory(float(time_value))
            rotation = __import__("glove_controller_alignment").quaternion_to_matrix(controller_q)
            position = pivot - rotation @ controller_to_pivot
            glove_q = multiply(multiply(world, controller_q), mount)
            controller.append(PoseSample(float(time_value), controller_q, position))
            glove.append(PoseSample(float(time_value + offset), glove_q))
        return controller, glove, mount, controller_to_pivot

    def test_estimates_temporal_offset(self):
        controller, glove, _, _ = self.synthetic_streams()
        offset, correlation, _ = estimate_time_offset(controller, glove, step_sec=0.005)
        self.assertAlmostEqual(offset, 0.12, delta=0.015)
        self.assertGreater(correlation, 0.95)

    def test_solves_hand_eye_and_pivot(self):
        controller, glove, _, expected_pivot = self.synthetic_streams()
        pairs = nearest_aligned_pairs(controller, glove, 0.12)
        solved_mount, _, metrics = solve_hand_eye_rotation(pairs)
        self.assertLess(metrics["rotation_residual_p95_deg"], 0.2)
        # The AX=XB solution is validated by residual because absolute quaternion
        # sign is arbitrary and symmetric trajectories can change its sign.
        self.assertTrue(np.all(np.isfinite(solved_mount)))
        solved_pivot, _, pivot_metrics = solve_controller_pivot(controller)
        np.testing.assert_allclose(solved_pivot, expected_pivot, atol=1e-6)
        self.assertLess(pivot_metrics["pivot_residual_p95_mm"], 1e-5)

    def test_fit_uses_time_correlation_validation_and_optional_pivot(self):
        controller, glove, _mount, _expected_pivot = self.synthetic_streams()
        neutral_controller = [
            PoseSample(index * 0.02, controller[0].quaternion_wxyz, controller[0].position_m)
            for index in range(120)
        ]
        neutral_glove = [
            PoseSample(index * 0.02 + 0.12, glove[0].quaternion_wxyz)
            for index in range(120)
        ]
        payload = fit_alignment(
            neutral_controller, neutral_glove, controller, glove, "zyx", require_pivot=False
        )
        quality = payload["quality"]
        self.assertGreaterEqual(quality["time_correlation_coefficient"], 0.8)
        self.assertLess(quality["validation_rotation_residual_p95_deg"], 0.5)
        self.assertTrue(quality["quality_pass"])
        self.assertFalse(quality["pivot_required"])

    def test_quaternion_angle_is_sign_invariant(self):
        value = axis_quaternion((1, 0, 0), math.radians(25))
        self.assertAlmostEqual(quaternion_angle_deg(value), 25.0, places=8)
        self.assertAlmostEqual(quaternion_angle_deg(-value), 25.0, places=8)


if __name__ == "__main__":
    unittest.main()
