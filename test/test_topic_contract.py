#!/usr/bin/env python3
"""
Regression checks for the ROS topic contract shared across vehicles.
"""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


class TestTopicContract(unittest.TestCase):
    ROOT = Path(__file__).resolve().parents[1]
    INSPECTION_MODEL = ROOT / 'models' / 'inspection_robot' / 'model.sdf'
    PRIUS_MODEL = ROOT / 'models' / 'prius_vehicle' / 'model.sdf'
    INSPECTION_LAUNCH = ROOT / 'launch' / 'inspection_world.launch.py'
    OFFROAD_LAUNCH = ROOT / 'launch' / 'offroad_world.launch.py'
    UDP_BRIDGE_SOURCE = ROOT / 'src' / 'gazebo_adapter' / 'udp_bridge.py'
    TOPIC_RELAY_SOURCE = ROOT / 'src' / 'gazebo_adapter' / 'topic_relay.py'
    MAKEFILE = ROOT / 'Makefile'
    CAMERA_TOPICS = (
        '/camera/main/image_raw',
        '/camera/left_side/image_raw',
        '/camera/left_mirror/image_raw',
        '/camera/right_side/image_raw',
        '/camera/right_mirror/image_raw',
        '/camera/rear/image_raw',
    )

    @staticmethod
    def _normalized_topic(topic):
        return topic if topic.startswith('/') else f'/{topic}'

    def test_vehicle_models_publish_same_core_sensor_topics(self):
        expectations = (
            (self.INSPECTION_MODEL, 'gps_sensor', '/mavros/global_position/global'),
            (self.INSPECTION_MODEL, 'imu_sensor', '/imu/data'),
            (self.PRIUS_MODEL, 'gps_sensor', '/mavros/global_position/global'),
            (self.PRIUS_MODEL, 'imu_sensor', '/imu/data'),
        )

        for model_path, sensor_name, expected_topic in expectations:
            with self.subTest(model=model_path.name, sensor=sensor_name):
                root = ET.parse(model_path).getroot()
                topic = root.findtext(f".//sensor[@name='{sensor_name}']/topic")
                self.assertEqual(topic, expected_topic)

        inspection_root = ET.parse(self.INSPECTION_MODEL).getroot()
        prius_root = ET.parse(self.PRIUS_MODEL).getroot()

        inspection_lidar = self._normalized_topic(
            inspection_root.findtext(".//sensor[@name='velodyne']/topic")
        )
        prius_lidar = self._normalized_topic(
            prius_root.findtext(".//sensor[@name='lidar_sensor']/topic")
        )

        self.assertEqual(inspection_lidar, '/lidar')
        self.assertEqual(prius_lidar, '/lidar')

    def test_world_launches_start_topic_relay(self):
        for launch_path in (self.INSPECTION_LAUNCH, self.OFFROAD_LAUNCH):
            with self.subTest(launch=launch_path.name):
                launch_text = launch_path.read_text()
                self.assertIn("executable='topic_relay'", launch_text)
                self.assertIn('period=13.0', launch_text)

    def test_world_launches_bridge_prius_camera_topics_and_video_streamer(self):
        for launch_path in (self.INSPECTION_LAUNCH, self.OFFROAD_LAUNCH):
            with self.subTest(launch=launch_path.name):
                launch_text = launch_path.read_text()
                for topic in self.CAMERA_TOPICS:
                    self.assertIn(topic, launch_text)
                self.assertIn("executable='video_streamer'", launch_text)
                self.assertIn('enable_video_streaming', launch_text)
                self.assertIn('linux_player_ip', launch_text)
                self.assertIn("prius_vehicle", launch_text)

    def test_udp_bridge_and_topic_relay_share_vehicle_contract(self):
        udp_bridge_text = self.UDP_BRIDGE_SOURCE.read_text()
        topic_relay_text = self.TOPIC_RELAY_SOURCE.read_text()

        for topic in (
            '/vehicle/cmd_vel',
            '/vehicle/cmd_drive',
            '/vehicle/cmd_gear',
            '/vehicle/odom',
            '/vehicle/gps',
        ):
            with self.subTest(topic=topic):
                self.assertIn(topic, udp_bridge_text)

        for topic in (
            '/vehicle/odom',
            '/ego/odometry',
            '/vehicle/imu',
            '/vehicle/gps',
            '/vehicle/lidar/points',
            '/cmd_vel',
            '/cmd_drive',
            '/cmd_gear',
        ):
            with self.subTest(topic=topic):
                self.assertIn(topic, topic_relay_text)

    def test_makefile_includes_topic_report_targets(self):
        makefile_text = self.MAKEFILE.read_text()

        self.assertIn('report-topics:', makefile_text)
        self.assertIn('report-topics-prius:', makefile_text)
        self.assertIn('/workspace/src/offroad_gazebo_integration/test_bridge.sh', makefile_text)
        self.assertIn('LINUX_PLAYER_IP', makefile_text)


if __name__ == '__main__':
    unittest.main()
