#!/usr/bin/env python3
"""
Unit tests for Gazebo adapter and UDP JSON protocol
"""

import json
import math
import time
import unittest

import numpy as np
from gazebo_adapter.adapter import (
    SimulationState,
    VehicleCommand,
    SensorData,
)


class TestSimulationState(unittest.TestCase):
    def test_initialization(self):
        state = SimulationState()
        self.assertEqual(state.timestamp, 0.0)
        self.assertTrue(isinstance(state.vehicle_pose, np.ndarray))
        self.assertEqual(len(state.vehicle_pose), 6)
        self.assertTrue(state.is_valid)


class TestVehicleCommand(unittest.TestCase):
    def test_initialization(self):
        cmd = VehicleCommand()
        self.assertEqual(cmd.linear_velocity, 0.0)
        self.assertEqual(cmd.angular_velocity, 0.0)
        self.assertEqual(cmd.steering_angle, 0.0)

    def test_set_values(self):
        cmd = VehicleCommand()
        cmd.linear_velocity = 5.0
        cmd.angular_velocity = 0.5
        cmd.steering_angle = 0.2
        self.assertEqual(cmd.linear_velocity, 5.0)
        self.assertEqual(cmd.angular_velocity, 0.5)
        self.assertEqual(cmd.steering_angle, 0.2)


class TestSensorData(unittest.TestCase):
    def test_initialization(self):
        data = SensorData()
        self.assertEqual(data.timestamp, 0.0)
        self.assertIsNone(data.imu)
        self.assertIsNone(data.gps)
        self.assertIsNone(data.lidar)
        self.assertIsNone(data.camera)


# ── JSON UDP Protocol Tests ──────────────────────────────────────


class TestJsonControlMessage(unittest.TestCase):
    """Verify control command JSON matches av-simulation's protocol."""

    REQUIRED_FIELDS = [
        'type', 'timestamp', 'throttle', 'steering', 'gear',
        'turn_left', 'turn_right', 'high_beam', 'low_beam', 'horn', 'hazard',
    ]

    def _make_control(self, **overrides):
        msg = {
            'type': 'control',
            'timestamp': int(time.time() * 1000),
            'throttle': 0.5,
            'steering': -0.3,
            'gear': 3,
            'turn_left': False,
            'turn_right': True,
            'high_beam': False,
            'low_beam': True,
            'horn': False,
            'hazard': False,
        }
        msg.update(overrides)
        return msg

    def test_serialization_roundtrip(self):
        original = self._make_control()
        encoded = json.dumps(original).encode('utf-8')
        decoded = json.loads(encoded)
        self.assertEqual(original, decoded)

    def test_required_fields_present(self):
        msg = self._make_control()
        for field in self.REQUIRED_FIELDS:
            self.assertIn(field, msg, f'Missing field: {field}')

    def test_type_is_control(self):
        msg = self._make_control()
        self.assertEqual(msg['type'], 'control')

    def test_throttle_range(self):
        for val in [-1.0, 0.0, 0.5, 1.0]:
            msg = self._make_control(throttle=val)
            data = json.loads(json.dumps(msg))
            self.assertAlmostEqual(data['throttle'], val)

    def test_non_control_type_ignored(self):
        """Bridge should ignore messages whose type != 'control'."""
        msg = self._make_control(type='heartbeat')
        self.assertNotEqual(msg['type'], 'control')


class TestJsonFeedbackMessage(unittest.TestCase):
    """Verify feedback JSON matches av-simulation's expected protocol."""

    REQUIRED_FIELDS = [
        'type', 'timestamp', 'speed', 'rpm', 'gps',
        'object_detected', 'vehicle_state',
    ]
    GPS_FIELDS = ['latitude', 'longitude', 'azimuth']

    def _make_feedback(self, **overrides):
        msg = {
            'type': 'feedback',
            'timestamp': int(time.time() * 1000),
            'speed': 3.5,
            'rpm': 1200,
            'gps': {
                'latitude': 31.7683,
                'longitude': 35.2137,
                'azimuth': 45.0,
            },
            'object_detected': False,
            'vehicle_state': 1,
        }
        msg.update(overrides)
        return msg

    def test_serialization_roundtrip(self):
        original = self._make_feedback()
        encoded = json.dumps(original).encode('utf-8')
        decoded = json.loads(encoded)
        self.assertEqual(original, decoded)

    def test_required_fields_present(self):
        msg = self._make_feedback()
        for field in self.REQUIRED_FIELDS:
            self.assertIn(field, msg, f'Missing field: {field}')

    def test_gps_subfields(self):
        msg = self._make_feedback()
        for field in self.GPS_FIELDS:
            self.assertIn(field, msg['gps'], f'Missing GPS field: {field}')

    def test_type_is_feedback(self):
        msg = self._make_feedback()
        self.assertEqual(msg['type'], 'feedback')

    def test_speed_non_negative(self):
        msg = self._make_feedback(speed=0.0)
        self.assertGreaterEqual(msg['speed'], 0.0)

    def test_azimuth_from_quaternion(self):
        """Azimuth computed from yaw should match expected value."""
        # quaternion for 90° yaw: (0, 0, sin(pi/4), cos(pi/4))
        from gazebo_adapter.udp_bridge import quaternion_to_yaw
        yaw = quaternion_to_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        self.assertAlmostEqual(math.degrees(yaw), 90.0, places=3)


if __name__ == '__main__':
    unittest.main()
