import signal
import subprocess
import unittest
from unittest.mock import patch

from quest3_camera_stream import stop_camera, stop_cameras


class StuckProcess:
    pid = 424242
    args = ["ffmpeg"]

    def poll(self):
        return None

    def wait(self, timeout=None):
        raise subprocess.TimeoutExpired(self.args, timeout)


class CameraCleanupTest(unittest.TestCase):
    @patch("quest3_camera_stream.os.killpg")
    def test_uninterruptible_ffmpeg_never_raises(self, killpg):
        self.assertFalse(stop_camera(StuckProcess()))
        self.assertEqual(
            [call.args[1] for call in killpg.call_args_list],
            [signal.SIGTERM, signal.SIGKILL],
        )

    @patch("quest3_camera_stream.stop_camera", return_value=True)
    def test_three_cameras_are_all_cleaned(self, stop):
        processes = [object(), object(), object()]
        self.assertEqual(stop_cameras(processes), [True, True, True])
        self.assertCountEqual([call.args[0] for call in stop.call_args_list], processes)


if __name__ == "__main__":
    unittest.main()
