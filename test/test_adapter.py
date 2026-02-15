#!/usr/bin/env python3
"""
Unit tests for Gazebo adapter
"""

import unittest
import numpy as np
from gazebo_adapter.adapter import (
    SimulationState,
    VehicleCommand,
    SensorData
)


class TestSimulationState(unittest.TestCase):
    """Test SimulationState data structure"""
    
    def test_initialization(self):
        """Test state initialization"""
        state = SimulationState()
        self.assertEqual(state.timestamp, 0.0)
        self.assertTrue(isinstance(state.vehicle_pose, np.ndarray))
        self.assertEqual(len(state.vehicle_pose), 6)
        self.assertTrue(state.is_valid)


class TestVehicleCommand(unittest.TestCase):
    """Test VehicleCommand data structure"""
    
    def test_initialization(self):
        """Test command initialization"""
        cmd = VehicleCommand()
        self.assertEqual(cmd.linear_velocity, 0.0)
        self.assertEqual(cmd.angular_velocity, 0.0)
        self.assertEqual(cmd.steering_angle, 0.0)
    
    def test_set_values(self):
        """Test setting command values"""
        cmd = VehicleCommand()
        cmd.linear_velocity = 5.0
        cmd.angular_velocity = 0.5
        cmd.steering_angle = 0.2
        
        self.assertEqual(cmd.linear_velocity, 5.0)
        self.assertEqual(cmd.angular_velocity, 0.5)
        self.assertEqual(cmd.steering_angle, 0.2)


class TestSensorData(unittest.TestCase):
    """Test SensorData data structure"""
    
    def test_initialization(self):
        """Test sensor data initialization"""
        data = SensorData()
        self.assertEqual(data.timestamp, 0.0)
        self.assertIsNone(data.imu)
        self.assertIsNone(data.gps)
        self.assertIsNone(data.lidar)
        self.assertIsNone(data.camera)


if __name__ == '__main__':
    unittest.main()
