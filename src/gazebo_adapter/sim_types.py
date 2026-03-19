#!/usr/bin/env python3
"""
Simulation data types that do not depend on ROS runtime imports.
"""

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np


@dataclass
class SimulationState:
    """Container for simulation state."""

    timestamp: float = 0.0
    vehicle_pose: np.ndarray = field(default_factory=lambda: np.zeros(6))
    vehicle_velocity: np.ndarray = field(default_factory=lambda: np.zeros(6))
    is_valid: bool = True


@dataclass
class VehicleCommand:
    """Container for vehicle commands."""

    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    steering_angle: float = 0.0
    timestamp: float = 0.0


@dataclass
class SensorData:
    """Container for sensor data."""

    timestamp: float = 0.0
    imu: Optional[Dict] = None
    gps: Optional[Dict] = None
    lidar: Optional[np.ndarray] = None
    camera: Optional[np.ndarray] = None
