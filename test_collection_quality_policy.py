import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from collect_workflow import (
    QUEST_MAX_PACKET_LOSS_SECONDS,
    ROBOT_MAX_SAMPLE_INTERVAL_SECONDS,
    Workflow,
    analyze_quest_quality,
    analyze_robot_quality,
)


def robot_payload(timestamp):
    samples = len(timestamp)
    values = np.zeros((samples, 14), dtype=np.float32)
    return {
        "observation_state": values,
        "observation_velocity": values,
        "observation_effort": values,
        "action": values,
    }


def quest_payload(ages):
    samples = len(ages)
    controller_state = np.zeros((samples, 26), dtype=np.float32)
    controller_state[:, 12] = 1.0
    controller_state[:, 25] = 1.0
    return {
        "quest_sequence": np.arange(samples, dtype=np.int64),
        "quest_packet_age_seconds": np.asarray(ages, dtype=np.float64),
        "quest_controller_state": controller_state,
        "teleop_source": np.asarray("meta_quest_3_touch"),
        "quest_schema": np.asarray("arx.quest3.controllers.v1"),
    }


class CollectionQualityPolicyTest(unittest.TestCase):
    def test_robot_accepts_40ms_but_not_more(self):
        self.assertEqual(ROBOT_MAX_SAMPLE_INTERVAL_SECONDS, 0.040)
        timestamp = np.asarray([0.0, 0.020, 0.060])
        self.assertTrue(analyze_robot_quality(robot_payload(timestamp), timestamp)["passed"])
        timestamp = np.asarray([0.0, 0.020, 0.061])
        self.assertFalse(analyze_robot_quality(robot_payload(timestamp), timestamp)["passed"])

    def test_quest_only_fails_after_three_seconds_without_packet(self):
        self.assertEqual(QUEST_MAX_PACKET_LOSS_SECONDS, 3.0)
        timestamp = np.asarray([0.0, 0.02, 0.04])
        report = analyze_quest_quality(quest_payload([0.0, 0.2, 3.0]), timestamp)
        self.assertTrue(report["passed"])
        self.assertNotIn("stale_samples", report)
        report = analyze_quest_quality(quest_payload([0.0, 0.2, 3.001]), timestamp)
        self.assertFalse(report["passed"])

    def test_failed_new_audit_is_deleted_before_review(self):
        with tempfile.TemporaryDirectory() as temporary:
            episode = Path(temporary) / "failed_episode"
            episode.mkdir()
            workflow = Workflow.__new__(Workflow)
            workflow.hardware = {"cameras": {}}
            workflow.pending_audits = lambda: [episode]
            with patch("collect_workflow.analyze_episode", return_value={"passed": False}):
                workflow.audit_pending_parallel()
            self.assertFalse(episode.exists())


if __name__ == "__main__":
    unittest.main()
