#!/usr/bin/env python3
"""
Unit tests for terrain management
"""

import unittest
import numpy as np
import tempfile
import os
from gazebo_adapter.terrain import TerrainManager, TerrainType


class TestTerrainManager(unittest.TestCase):
    """Test TerrainManager functionality"""
    
    def test_heightmap_generation(self):
        """Test heightmap generation"""
        heightmap = TerrainManager.generate_random_heightmap(
            size=(128, 128),
            height_range=(0, 10),
            roughness=0.5,
            seed=42
        )
        
        self.assertEqual(heightmap.shape, (128, 128))
        self.assertTrue(heightmap.min() >= 0)
        self.assertTrue(heightmap.max() <= 255)
    
    def test_heightmap_save(self):
        """Test heightmap saving"""
        heightmap = TerrainManager.generate_random_heightmap(
            size=(64, 64),
            seed=42
        )
        
        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
            filepath = f.name
        
        try:
            TerrainManager.save_heightmap(heightmap, filepath)
            self.assertTrue(os.path.exists(filepath))
            self.assertTrue(os.path.getsize(filepath) > 0)
        finally:
            if os.path.exists(filepath):
                os.remove(filepath)
    
    def test_terrain_sdf_creation(self):
        """Test SDF generation"""
        sdf = TerrainManager.create_terrain_sdf(
            name='test_terrain',
            heightmap_path='/tmp/test.png',
            size=(100, 100, 20),
            terrain_type=TerrainType.DIRT
        )
        
        self.assertIn('test_terrain', sdf)
        self.assertIn('heightmap', sdf)
        self.assertIn('/tmp/test.png', sdf)
        self.assertIn('100 100 20', sdf)
    
    def test_terrain_types(self):
        """Test predefined terrain types"""
        self.assertIn('friction', TerrainType.DIRT)
        self.assertIn('friction', TerrainType.SAND)
        self.assertIn('friction', TerrainType.MUD)
        
        # Sand should have lower friction than dirt
        self.assertLess(
            TerrainType.SAND['friction'],
            TerrainType.DIRT['friction']
        )
    
    def test_world_generation(self):
        """Test complete world generation"""
        manager = TerrainManager()
        
        config = {
            'heightmap_size': (128, 128),
            'height_range': (0, 10),
            'roughness': 0.5,
            'size': (100, 100, 10),
            'type': 'dirt',
            'seed': 42,
            'heightmap_path': '/tmp/test_terrain.png'
        }
        
        with tempfile.NamedTemporaryFile(suffix='.sdf', delete=False) as f:
            output_path = f.name
        
        try:
            world_sdf = manager.generate_terrain_world(
                world_name='test_world',
                terrain_config=config,
                output_path=output_path
            )
            
            self.assertIn('test_world', world_sdf)
            self.assertIn('terrain', world_sdf)
            self.assertTrue(os.path.exists(output_path))
        finally:
            if os.path.exists(output_path):
                os.remove(output_path)
            if os.path.exists('/tmp/test_terrain.png'):
                os.remove('/tmp/test_terrain.png')


if __name__ == '__main__':
    unittest.main()
