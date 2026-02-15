#!/usr/bin/env python3
"""
Terrain Management Utilities

Utilities for creating and managing off-road terrains in Gazebo simulation.
"""

from typing import Tuple, Optional
import numpy as np
from PIL import Image
import xml.etree.ElementTree as ET


class TerrainType:
    """Predefined terrain material types with physics properties"""
    
    DIRT = {
        'name': 'dirt',
        'friction': 0.8,
        'friction2': 0.7,
        'restitution': 0.1,
        'color': [0.6, 0.4, 0.2, 1.0]
    }
    
    SAND = {
        'name': 'sand',
        'friction': 0.6,
        'friction2': 0.5,
        'restitution': 0.05,
        'color': [0.9, 0.8, 0.6, 1.0]
    }
    
    GRAVEL = {
        'name': 'gravel',
        'friction': 0.7,
        'friction2': 0.65,
        'restitution': 0.2,
        'color': [0.5, 0.5, 0.5, 1.0]
    }
    
    MUD = {
        'name': 'mud',
        'friction': 0.4,
        'friction2': 0.3,
        'restitution': 0.0,
        'color': [0.3, 0.25, 0.15, 1.0]
    }
    
    ROCK = {
        'name': 'rock',
        'friction': 0.9,
        'friction2': 0.85,
        'restitution': 0.4,
        'color': [0.4, 0.4, 0.4, 1.0]
    }


class TerrainManager:
    """
    Manages terrain generation and configuration for Gazebo off-road simulation.
    
    Provides methods to:
    - Generate heightmap terrains
    - Configure terrain physics properties
    - Create SDF terrain models
    """
    
    def __init__(self):
        self.terrain_models = []
    
    @staticmethod
    def generate_random_heightmap(
        size: Tuple[int, int] = (256, 256),
        height_range: Tuple[float, float] = (0.0, 10.0),
        roughness: float = 0.5,
        seed: Optional[int] = None
    ) -> np.ndarray:
        """
        Generate a random heightmap using Perlin-like noise.
        
        Args:
            size: (width, height) in pixels
            height_range: (min, max) elevation in meters
            roughness: Terrain roughness factor (0.0-1.0)
            seed: Random seed for reproducibility
            
        Returns:
            2D numpy array of heights (normalized to 0-255 for PNG export)
        """
        if seed is not None:
            np.random.seed(seed)
        
        # Generate multi-scale noise
        width, height = size
        heightmap = np.zeros((height, width))
        
        scales = [1, 2, 4, 8, 16]
        weights = [1.0, 0.5, 0.25, 0.125, 0.0625]
        
        for scale, weight in zip(scales, weights):
            noise = np.random.rand(height // scale + 1, width // scale + 1)
            # Simple upsampling (would use proper interpolation in production)
            upsampled = np.repeat(np.repeat(noise, scale, axis=0), scale, axis=1)
            upsampled = upsampled[:height, :width]
            heightmap += upsampled * weight * roughness
        
        # Normalize
        heightmap = (heightmap - heightmap.min()) / (heightmap.max() - heightmap.min())
        
        # Scale to height range
        min_h, max_h = height_range
        heightmap = heightmap * (max_h - min_h) + min_h
        
        # Convert to 0-255 range for PNG
        heightmap_png = ((heightmap - min_h) / (max_h - min_h) * 255).astype(np.uint8)
        
        return heightmap_png
    
    @staticmethod
    def save_heightmap(heightmap: np.ndarray, filepath: str):
        """
        Save heightmap as grayscale PNG.
        
        Args:
            heightmap: 2D array of heights (0-255)
            filepath: Output PNG file path
        """
        img = Image.fromarray(heightmap, mode='L')
        img.save(filepath)
    
    @staticmethod
    def create_terrain_sdf(
        name: str,
        heightmap_path: str,
        size: Tuple[float, float, float],
        terrain_type: dict = TerrainType.DIRT,
        position: Tuple[float, float, float] = (0, 0, 0)
    ) -> str:
        """
        Generate SDF XML for heightmap terrain.
        
        Args:
            name: Terrain model name
            heightmap_path: Path to heightmap PNG file
            size: (length, width, max_height) in meters
            terrain_type: Terrain material properties
            position: (x, y, z) position
            
        Returns:
            SDF XML string
        """
        length, width, max_height = size
        x, y, z = position
        
        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <heightmap>
            <uri>{heightmap_path}</uri>
            <size>{length} {width} {max_height}</size>
            <pos>{x} {y} {z}</pos>
          </heightmap>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{terrain_type['friction']}</mu>
              <mu2>{terrain_type['friction2']}</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>{terrain_type['restitution']}</restitution_coefficient>
          </bounce>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <heightmap>
            <uri>{heightmap_path}</uri>
            <size>{length} {width} {max_height}</size>
            <pos>{x} {y} {z}</pos>
          </heightmap>
        </geometry>
        <material>
          <ambient>{' '.join(map(str, terrain_type['color']))}</ambient>
          <diffuse>{' '.join(map(str, terrain_type['color']))}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        return sdf
    
    def generate_terrain_world(
        self,
        world_name: str,
        terrain_config: dict,
        vehicle_spawn: Tuple[float, float, float] = (0, 0, 0.5),
        output_path: str = None
    ) -> str:
        """
        Generate complete Gazebo world with terrain.
        
        Args:
            world_name: World name
            terrain_config: Terrain configuration dict
            vehicle_spawn: Initial vehicle position
            output_path: Output SDF file path (if None, returns string)
            
        Returns:
            World SDF XML string
        """
        # Generate heightmap
        heightmap = self.generate_random_heightmap(
            size=terrain_config.get('heightmap_size', (256, 256)),
            height_range=terrain_config.get('height_range', (0, 10)),
            roughness=terrain_config.get('roughness', 0.5),
            seed=terrain_config.get('seed')
        )
        
        # Save heightmap
        heightmap_path = terrain_config.get('heightmap_path', '/tmp/terrain.png')
        self.save_heightmap(heightmap, heightmap_path)
        
        # Get terrain type
        terrain_type_name = terrain_config.get('type', 'dirt')
        terrain_type = getattr(TerrainType, terrain_type_name.upper(), TerrainType.DIRT)
        
        # Create terrain SDF
        terrain_sdf = self.create_terrain_sdf(
            name='terrain',
            heightmap_path=heightmap_path,
            size=terrain_config.get('size', (100, 100, 20)),
            terrain_type=terrain_type
        )
        
        # Complete world SDF
        vx, vy, vz = vehicle_spawn
        world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{world_name}">
    <!-- Physics -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Plugins -->
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Terrain -->
    {terrain_sdf}
    
    <!-- Vehicle spawn point marker (optional) -->
    <model name="spawn_marker">
      <static>true</static>
      <pose>{vx} {vy} {vz} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>"""
        
        if output_path:
            with open(output_path, 'w') as f:
                f.write(world_sdf)
        
        return world_sdf


def main():
    """Example: Generate random off-road terrain world"""
    manager = TerrainManager()
    
    terrain_config = {
        'heightmap_size': (512, 512),
        'height_range': (0, 15),
        'roughness': 0.7,
        'size': (200, 200, 15),
        'type': 'dirt',
        'seed': 42,
        'heightmap_path': '/tmp/offroad_terrain.png'
    }
    
    world_sdf = manager.generate_terrain_world(
        world_name='offroad_test',
        terrain_config=terrain_config,
        vehicle_spawn=(0, 0, 1.0),
        output_path='/tmp/offroad_world.sdf'
    )
    
    print("Generated terrain world at /tmp/offroad_world.sdf")
    print(f"Heightmap saved at {terrain_config['heightmap_path']}")


if __name__ == '__main__':
    main()
