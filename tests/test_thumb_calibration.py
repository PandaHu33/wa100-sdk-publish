import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "examples"))

from thumb_calibration import JOINT_INDICES, calibration_payload, fit_candidate  # noqa: E402


def sample(common=(0.0, 0.0, 0.0), yaw=0.0, pitch=0.0):
    value = np.zeros((len(JOINT_INDICES), 3), dtype=float)
    value += np.asarray(common, dtype=float)
    value[0, 1] += yaw
    value[2, 2] += pitch
    return value


def repeated(value, count=30):
    return [np.array(value, copy=True) for _ in range(count)]


class ThumbCalibrationTests(unittest.TestCase):
    def synthetic_states(self):
        yaw_sweep = [sample(yaw=30.0 * (0.5 + 0.5 * np.sin(index * 0.2))) for index in range(90)]
        pitch_sweep = [sample(pitch=35.0 * (0.5 + 0.5 * np.sin(index * 0.2))) for index in range(90)]
        return {
            "neutral": repeated(sample()),
            "yaw_open": repeated(sample(yaw=0.0)),
            "yaw_close": repeated(sample(yaw=30.0)),
            "yaw_sweep": yaw_sweep,
            "pitch_open": repeated(sample(pitch=0.0)),
            "pitch_close": repeated(sample(pitch=35.0)),
            "pitch_sweep": pitch_sweep,
            "combined": [sample(yaw=index / 3.0, pitch=index / 2.5) for index in range(60)],
            "neutral_validate": repeated(sample(), 50),
        }

    def test_fit_separates_actions_with_fixed_wrist(self):
        fit = fit_candidate(self.synthetic_states(), "xyz")
        metrics = fit["metrics"]
        self.assertTrue(metrics["quality_pass"], metrics)
        self.assertLessEqual(metrics["neutral_validation_yaw_target_range"], 1e-5)
        self.assertLessEqual(metrics["neutral_validation_pitch_target_range"], 1e-5)
        self.assertGreaterEqual(metrics["yaw_sweep_target_range"], 1900.0)
        self.assertGreaterEqual(metrics["pitch_sweep_target_range"], 1900.0)
        self.assertLessEqual(metrics["yaw_sweep_pitch_crosstalk"], 1e-5)
        self.assertLessEqual(metrics["pitch_sweep_yaw_crosstalk"], 1e-5)

    def test_payload_has_runtime_schema(self):
        states = self.synthetic_states()
        fit = fit_candidate(states, "xyz")
        payload = calibration_payload("right", fit, {key: len(value) for key, value in states.items()})
        self.assertEqual(payload["schema_version"], 2)
        self.assertEqual(payload["hand"], "right")
        self.assertEqual(payload["model"], "single_thumb_fixed_wrist")
        self.assertEqual(len(payload["yaw_axis"]), 9)
        self.assertEqual(np.asarray(payload["demix_matrix"]).shape, (2, 2))
        self.assertTrue(payload["quality"]["quality_pass"])


if __name__ == "__main__":
    unittest.main()
