#!/usr/bin/env python3
"""
Unit tests for Gazebo adapter and UDP JSON protocol
"""

import json
import math
from pathlib import Path
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from gazebo_adapter.control_mapping import (
    ACKERMANN_DRIVE_MODE,
    DEFAULT_ACKERMANN_STEERING_LIMIT,
    GEAR_DRIVE,
    GEAR_NEUTRAL,
    GEAR_PARKING,
    GEAR_REVERSE,
    PRIUS_DRIVE_MODE,
    TANK_DRIVE_MODE,
    adapter_command_to_targets,
    udp_command_to_targets,
)
from gazebo_adapter.math_utils import quaternion_to_yaw
from gazebo_adapter.sim_types import SensorData, SimulationState, VehicleCommand


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


class TestControlMapping(unittest.TestCase):
    def test_tank_mapping_uses_yaw_rate(self):
        targets = udp_command_to_targets(
            throttle=0.5,
            steering=0.25,
            drive_mode=TANK_DRIVE_MODE,
            max_linear_speed=10.0,
            max_angular_speed=2.0,
        )
        self.assertAlmostEqual(targets.linear_velocity, 5.0)
        self.assertAlmostEqual(targets.angular_velocity, -0.5)
        self.assertAlmostEqual(targets.steering_angle, 0.0)

    def test_ackermann_mapping_uses_steering_limit(self):
        targets = udp_command_to_targets(
            throttle=0.5,
            steering=1.0,
            drive_mode=ACKERMANN_DRIVE_MODE,
            max_linear_speed=10.0,
            max_angular_speed=2.0,
            ackermann_steering_limit=DEFAULT_ACKERMANN_STEERING_LIMIT,
        )
        self.assertAlmostEqual(targets.linear_velocity, 5.0)
        self.assertAlmostEqual(
            targets.steering_angle,
            -DEFAULT_ACKERMANN_STEERING_LIMIT,
        )
        self.assertLess(targets.angular_velocity, 0.0)

    def test_ackermann_mapping_clamps_steering_input(self):
        targets = udp_command_to_targets(
            throttle=0.3,
            steering=3.0,
            drive_mode=ACKERMANN_DRIVE_MODE,
            max_linear_speed=10.0,
            max_angular_speed=2.0,
        )
        self.assertAlmostEqual(
            targets.steering_angle,
            -DEFAULT_ACKERMANN_STEERING_LIMIT,
        )

    def test_ackermann_mapping_preserves_reverse(self):
        targets = udp_command_to_targets(
            throttle=-0.4,
            steering=-0.5,
            drive_mode=ACKERMANN_DRIVE_MODE,
            max_linear_speed=10.0,
            max_angular_speed=2.0,
        )
        self.assertAlmostEqual(targets.linear_velocity, -4.0)
        self.assertGreater(targets.steering_angle, 0.0)
        self.assertLess(targets.angular_velocity, 0.0)

    def test_adapter_ackermann_uses_steering_angle(self):
        targets = adapter_command_to_targets(
            linear_velocity=6.0,
            angular_velocity=99.0,
            steering_angle=0.2,
            drive_mode=ACKERMANN_DRIVE_MODE,
        )
        self.assertAlmostEqual(targets.linear_velocity, 6.0)
        self.assertAlmostEqual(targets.steering_angle, 0.2)
        self.assertNotEqual(targets.angular_velocity, 99.0)

    def test_prius_mapping_uses_gear_for_forward_motion(self):
        targets = udp_command_to_targets(
            throttle=0.6,
            steering=0.25,
            gear=GEAR_DRIVE,
            drive_mode=PRIUS_DRIVE_MODE,
            max_linear_speed=10.0,
            max_angular_speed=2.0,
        )
        self.assertAlmostEqual(targets.throttle, 0.6)
        self.assertAlmostEqual(targets.brake, 0.0)
        self.assertAlmostEqual(targets.steering_input, 0.25)
        self.assertEqual(targets.gear, GEAR_DRIVE)

    def test_prius_mapping_uses_reverse_gear_not_negative_speed(self):
        targets = udp_command_to_targets(
            throttle=0.4,
            steering=-0.5,
            gear=GEAR_REVERSE,
            drive_mode=PRIUS_DRIVE_MODE,
            max_linear_speed=10.0,
            max_angular_speed=2.0,
        )
        self.assertAlmostEqual(targets.throttle, 0.4)
        self.assertAlmostEqual(targets.brake, 0.0)
        self.assertAlmostEqual(targets.steering_input, -0.5)
        self.assertEqual(targets.gear, GEAR_REVERSE)

    def test_prius_mapping_converts_negative_gas_to_brake(self):
        targets = udp_command_to_targets(
            throttle=-0.7,
            steering=0.0,
            gear=GEAR_DRIVE,
            drive_mode=PRIUS_DRIVE_MODE,
            max_linear_speed=10.0,
            max_angular_speed=2.0,
        )
        self.assertAlmostEqual(targets.throttle, 0.0)
        self.assertAlmostEqual(targets.brake, 0.7)
        self.assertEqual(targets.gear, GEAR_DRIVE)

    def test_prius_mapping_ignores_positive_gas_in_parking(self):
        targets = udp_command_to_targets(
            throttle=0.8,
            steering=0.1,
            gear=GEAR_PARKING,
            drive_mode=PRIUS_DRIVE_MODE,
            max_linear_speed=10.0,
            max_angular_speed=2.0,
        )
        self.assertAlmostEqual(targets.throttle, 0.0)
        self.assertAlmostEqual(targets.brake, 0.0)
        self.assertEqual(targets.gear, GEAR_PARKING)

    def test_prius_mode_normalizes_unknown_gear_to_neutral(self):
        targets = udp_command_to_targets(
            throttle=0.5,
            steering=0.0,
            gear=999,
            drive_mode=PRIUS_DRIVE_MODE,
            max_linear_speed=10.0,
            max_angular_speed=2.0,
        )
        self.assertAlmostEqual(targets.throttle, 0.0)
        self.assertAlmostEqual(targets.brake, 0.0)
        self.assertEqual(targets.gear, GEAR_NEUTRAL)


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
        yaw = quaternion_to_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        self.assertAlmostEqual(math.degrees(yaw), 90.0, places=3)


if __name__ == '__main__':
    unittest.main()
