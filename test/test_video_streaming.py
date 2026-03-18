#!/usr/bin/env python3
"""
Regression checks for the Prius Linux Player video streaming contract.
"""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from gazebo_adapter.video_streaming import (
    CAMERA_STREAM_CONFIGS,
    DEFAULT_H265_PAYLOAD_TYPE,
)


class TestVideoStreamingContract(unittest.TestCase):
    def test_linux_player_camera_names_and_ports_match_expected_contract(self):
        expected = (
            ('main', '/camera/main/image_raw', 1230),
            ('left_side', '/camera/left_side/image_raw', 1231),
            ('right_side', '/camera/right_side/image_raw', 1232),
            ('rear', '/camera/rear/image_raw', 1233),
            ('left_mirror', '/camera/left_mirror/image_raw', 1234),
            ('right_mirror', '/camera/right_mirror/image_raw', 1235),
        )

        observed = tuple(
            (config.name, config.topic, config.port)
            for config in CAMERA_STREAM_CONFIGS
        )
        self.assertEqual(observed, expected)

    def test_h265_rtp_payload_type_is_locked(self):
        self.assertEqual(DEFAULT_H265_PAYLOAD_TYPE, 96)


if __name__ == '__main__':
    unittest.main()
