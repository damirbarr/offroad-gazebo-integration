#!/usr/bin/env python3
"""
Regression checks for the Prius Gazebo model.
"""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


class TestPriusModel(unittest.TestCase):
    MODEL_PATH = (
        Path(__file__).resolve().parents[1]
        / 'models'
        / 'prius_vehicle'
        / 'model.sdf'
    )
    WHEEL_LINK_NAMES = (
        'front_left_wheel',
        'front_right_wheel',
        'rear_left_wheel',
        'rear_right_wheel',
    )
    WHEEL_JOINT_NAMES = (
        'front_left_wheel_joint',
        'front_right_wheel_joint',
        'rear_left_wheel_joint',
        'rear_right_wheel_joint',
    )
    EXPECTED_VISUAL_POSE = '0 0 0 0 -1.5708 0'
    EXPECTED_WHEEL_JOINT_AXIS = '0 0 1'
    CAMERA_EXPECTATIONS = {
        'main_camera': ('1.20 0.00 1.45 0 0 0.0000', '/camera/main/image_raw'),
        'left_side_camera': ('0.25 0.88 1.35 0 0 1.5708', '/camera/left_side/image_raw'),
        'left_mirror_camera': ('0.95 0.93 1.32 0 0 2.3562', '/camera/left_mirror/image_raw'),
        'right_side_camera': ('0.25 -0.88 1.35 0 0 -1.5708', '/camera/right_side/image_raw'),
        'right_mirror_camera': ('0.95 -0.93 1.32 0 0 -2.3562', '/camera/right_mirror/image_raw'),
        'rear_camera': ('-1.95 0.00 1.40 0 0 3.1416', '/camera/rear/image_raw'),
    }
    EXPECTED_CAMERA_UPDATE_RATE = '15'
    EXPECTED_CAMERA_RESOLUTION = ('1280', '720')
    EXPECTED_CAMERA_FOV = '1.57'
    EXPECTED_CAMERA_FORMAT = 'R8G8B8'

    @classmethod
    def setUpClass(cls):
        cls.tree = ET.parse(cls.MODEL_PATH)
        cls.root = cls.tree.getroot()

    def test_wheel_meshes_have_explicit_visual_rotation(self):
        for link_name in self.WHEEL_LINK_NAMES:
            with self.subTest(link_name=link_name):
                pose = self.root.findtext(
                    f".//link[@name='{link_name}']/visual[@name='visual']/pose"
                )
                self.assertEqual(pose, self.EXPECTED_VISUAL_POSE)

    def test_prius_drive_plugin_uses_cmd_drive_and_cmd_gear_topics(self):
        plugin = self.root.find(".//plugin[@name='offroad_gazebo_integration::PriusDriveSystem']")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.findtext('drive_topic'), '/cmd_drive')
        self.assertEqual(plugin.findtext('gear_topic'), '/cmd_gear')

    def test_wheel_joints_spin_about_the_link_z_axis(self):
        for joint_name in self.WHEEL_JOINT_NAMES:
            with self.subTest(joint_name=joint_name):
                axis = self.root.findtext(f".//joint[@name='{joint_name}']/axis/xyz")
                self.assertEqual(axis, self.EXPECTED_WHEEL_JOINT_AXIS)

    def test_odometry_publisher_uses_existing_odom_topic(self):
        topic = self.root.findtext(
            ".//plugin[@name='ignition::gazebo::systems::OdometryPublisher']/odom_topic"
        )
        self.assertEqual(topic, '/odom')

    def test_native_ackermann_plugin_is_not_attached(self):
        plugin = self.root.find(
            ".//plugin[@name='ignition::gazebo::systems::AckermannSteering']"
        )
        self.assertIsNone(plugin)

    def test_prius_model_has_expected_camera_topics_and_poses(self):
        for sensor_name, (expected_pose, expected_topic) in self.CAMERA_EXPECTATIONS.items():
            with self.subTest(sensor_name=sensor_name):
                sensor = self.root.find(f".//sensor[@name='{sensor_name}']")
                self.assertIsNotNone(sensor)
                self.assertEqual(sensor.findtext('pose'), expected_pose)
                self.assertEqual(sensor.findtext('topic'), expected_topic)
                self.assertEqual(sensor.findtext('update_rate'), self.EXPECTED_CAMERA_UPDATE_RATE)
                self.assertEqual(sensor.findtext('camera/horizontal_fov'), self.EXPECTED_CAMERA_FOV)
                self.assertEqual(sensor.findtext('camera/image/width'), self.EXPECTED_CAMERA_RESOLUTION[0])
                self.assertEqual(sensor.findtext('camera/image/height'), self.EXPECTED_CAMERA_RESOLUTION[1])
                self.assertEqual(sensor.findtext('camera/image/format'), self.EXPECTED_CAMERA_FORMAT)


if __name__ == '__main__':
    unittest.main()
