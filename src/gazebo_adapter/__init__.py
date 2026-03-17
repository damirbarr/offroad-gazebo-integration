"""
Gazebo Off-Road Simulation Adapter for Ottopia AV Simulation.
"""

from importlib import import_module

__version__ = "0.1.0"
__all__ = ["GazeboAdapter", "GazeboROSBridge", "TerrainManager"]


def __getattr__(name):
    """Lazily import ROS-dependent modules so pure unit tests can run locally."""
    if name == "GazeboAdapter":
        return import_module(".adapter", __name__).GazeboAdapter
    if name == "GazeboROSBridge":
        return import_module(".bridge", __name__).GazeboROSBridge
    if name == "TerrainManager":
        return import_module(".terrain", __name__).TerrainManager
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
