from dataclasses import replace
import json
import socket
import time
import unittest

import numpy as np

from quest3_input import (
    ControllerState,
    QuestSnapshot,
    Quest3Receiver,
    SCHEMA,
    WorkflowButtonMapper,
    decode_datagram,
)


class QuestInputTest(unittest.TestCase):
    def packet(self, sequence=1):
        side = {
            "position_m": [1, 2, 3],
            "orientation_xyzw": [0, 0, 0, 1],
            "tracking": True,
            "thumbstick_xy": [0.25, -0.5],
            "buttons": {"grip": False, "trigger": 0.2},
        }
        return json.dumps({"schema": SCHEMA, "sequence": sequence, "left": side, "right": side}).encode()

    def test_decode(self):
        state = decode_datagram(self.packet(), received_monotonic=10.0)
        self.assertEqual(state.sequence, 1)
        self.assertEqual(state.left.position, (1.0, 2.0, 3.0))
        self.assertAlmostEqual(state.right.trigger, 0.2)
        self.assertEqual(state.left.thumbstick_xy, (0.25, -0.5))

    def test_thumbstick_triggers_once_until_recentered(self):
        mapper = WorkflowButtonMapper()
        pushed = QuestSnapshot(
            sequence=1,
            received_monotonic=10.0,
            right=ControllerState(thumbstick_xy=(0.0, 0.9), tracking=True),
        )
        self.assertEqual(mapper.update("采集阶段等待", pushed, now=10.0), "s")
        self.assertIsNone(mapper.update("采集阶段等待", pushed, now=10.1))
        centered = replace(
            pushed,
            sequence=2,
            received_monotonic=10.2,
            right=ControllerState(thumbstick_xy=(0.0, 0.0), tracking=True),
        )
        self.assertIsNone(mapper.update("采集阶段等待", centered, now=10.2))
        pushed_again = replace(pushed, sequence=3, received_monotonic=10.3)
        self.assertEqual(mapper.update("采集阶段等待", pushed_again, now=10.3), "s")

    def test_thumbstick_review_mapping(self):
        mapper = WorkflowButtonMapper()
        snapshot = QuestSnapshot(
            sequence=1,
            received_monotonic=10.0,
            left=ControllerState(thumbstick_xy=(-0.9, 0.0), tracking=True),
        )
        self.assertEqual(mapper.update("审阅阶段", snapshot, now=10.0), "p")

    def test_rejects_bad_quaternion(self):
        payload = json.loads(self.packet())
        payload["left"]["orientation_xyzw"] = [0, 0, 0, 0]
        with self.assertRaises(ValueError):
            decode_datagram(json.dumps(payload).encode(), received_monotonic=10.0)

    def test_long_press_is_one_shot(self):
        mapper = WorkflowButtonMapper()
        snapshot = QuestSnapshot(
            sequence=1,
            received_monotonic=10.0,
            right=ControllerState(primary=True, tracking=True),
        )
        self.assertIsNone(mapper.update("采集阶段等待", snapshot, now=10.0))
        held = replace(snapshot, sequence=2, received_monotonic=10.8)
        self.assertEqual(mapper.update("采集阶段等待", held, now=10.8), "s")
        held = replace(snapshot, sequence=3, received_monotonic=11.0)
        self.assertIsNone(mapper.update("采集阶段等待", held, now=11.0))

    def test_grip_blocks_workflow_action(self):
        mapper = WorkflowButtonMapper()
        snapshot = QuestSnapshot(
            sequence=1,
            received_monotonic=10.0,
            left=ControllerState(grip=True, tracking=True),
            right=ControllerState(primary=True, tracking=True),
        )
        self.assertIsNone(mapper.update("采集阶段等待", snapshot, now=10.0))
        self.assertIsNone(mapper.update("采集阶段等待", snapshot, now=12.0))

    def test_shutdown_chord_suppresses_short_actions(self):
        mapper = WorkflowButtonMapper()
        snapshot = QuestSnapshot(
            sequence=1,
            received_monotonic=10.0,
            left=ControllerState(primary=True, tracking=True),
            right=ControllerState(secondary=True, tracking=True),
        )
        self.assertIsNone(mapper.update("采集阶段等待", snapshot, now=10.0))
        held = replace(snapshot, sequence=2, received_monotonic=10.8)
        self.assertIsNone(mapper.update("采集阶段等待", held, now=10.8))
        held = replace(snapshot, sequence=3, received_monotonic=12.1)
        self.assertEqual(mapper.update("采集阶段等待", held, now=12.1), "q")

    def test_all_workflow_stages_are_controller_complete(self):
        stick_cases = {
            "采集阶段等待": {
                (0.0, 0.9): "s", (0.0, -0.9): "v",
                (-0.9, 0.0): "o", (0.9, 0.0): "r",
            },
            "采集中": {(0.0, -0.9): "e"},
            "审阅阶段": {
                (0.0, 0.9): "a", (0.0, -0.9): "x",
                (-0.9, 0.0): "p", (0.9, 0.0): "b",
            },
        }
        for state, cases in stick_cases.items():
            for stick, expected in cases.items():
                with self.subTest(state=state, stick=stick):
                    mapper = WorkflowButtonMapper()
                    snapshot = QuestSnapshot(
                        sequence=1,
                        received_monotonic=10.0,
                        right=ControllerState(thumbstick_xy=stick, tracking=True),
                    )
                    self.assertEqual(mapper.update(state, snapshot, now=10.0), expected)

    def test_shutdown_chord_is_available_in_all_interactive_stages(self):
        for state in ("采集阶段等待", "采集中", "审阅阶段"):
            with self.subTest(state=state, command="q"):
                mapper = WorkflowButtonMapper()
                pressed = QuestSnapshot(
                    sequence=1,
                    received_monotonic=10.0,
                    left=ControllerState(primary=True, tracking=True),
                    right=ControllerState(secondary=True, tracking=True),
                )
                self.assertIsNone(mapper.update(state, pressed, now=10.0))
                held = replace(pressed, sequence=2, received_monotonic=12.1)
                self.assertEqual(mapper.update(state, held, now=12.1), "q")

    def test_menu_hold_captures_one_second_forward_displacement(self):
        mapper = WorkflowButtonMapper()
        started = QuestSnapshot(
            sequence=1,
            received_monotonic=10.0,
            left=ControllerState(
                position=(0.10, 1.20, -0.30), menu=True, tracking=True
            ),
            right=ControllerState(tracking=True),
        )
        self.assertIsNone(mapper.update("采集阶段等待", started, now=10.0))
        finished = replace(
            started,
            sequence=2,
            received_monotonic=11.0,
            left=replace(started.left, position=(0.25, 1.22, -0.34)),
        )
        self.assertEqual(mapper.update("采集阶段等待", finished, now=11.0), "c")
        np.testing.assert_allclose(
            mapper.consume_forward_displacement(), (0.15, 0.02, -0.04)
        )

    def test_stale_packet_blocks_action(self):
        mapper = WorkflowButtonMapper()
        snapshot = QuestSnapshot(
            sequence=1,
            received_monotonic=1.0,
            right=ControllerState(primary=True, tracking=True),
        )
        self.assertIsNone(mapper.update("采集阶段等待", snapshot, now=2.0))

    def test_udp_receiver_latches_first_sender_and_updates(self):
        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except PermissionError:
            self.skipTest("runtime sandbox forbids local UDP sockets")
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        receiver = Quest3Receiver(host="127.0.0.1", port=port)
        receiver.start()
        self.assertTrue(receiver.ready.wait(timeout=1.0))
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(self.packet(sequence=7), ("127.0.0.1", port))
            deadline = time.monotonic() + 1.0
            while receiver.snapshot().sequence != 7 and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(receiver.snapshot().sequence, 7)
            self.assertEqual(receiver.sender, "127.0.0.1")
        finally:
            sender.close()
            receiver.close()


if __name__ == "__main__":
    unittest.main()
