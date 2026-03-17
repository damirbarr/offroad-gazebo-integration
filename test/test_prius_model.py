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


if __name__ == '__main__':
    unittest.main()
