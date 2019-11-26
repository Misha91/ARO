from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import os
import pickle
import tempfile
import unittest
from voxel_map import VoxelMap


class TestVoxelMap(unittest.TestCase):
    def setUp(self):
        self.map = VoxelMap()
        self.map.voxel_size = 0.5
        self.map.free_update = -1.0
        self.map.hit_update = 2.0
        self.map.occupancy_threshold = 0.0
        self.assertIsNotNone(self.map)

    def test_setget_voxels(self):
        n = 5
        x0 = 10 * np.random.rand(3, n)
        l0 = np.zeros((n,), dtype=np.float64)
        v0 = 2 * np.random.rand(n) - 1
        print(x0, l0, v0)
        self.map.set_voxels(x0, l0, v0)
        print(x0, l0, v0)

        v1 = self.map.get_voxels(x0, l0)
        for el0, el1 in zip(v0.tolist(), v1.tolist()):
            self.assertAlmostEqual(el0, el1)

        [x2, l2, v2] = self.map.get_voxels()

        v3 = self.map.get_voxels(x2, l2)
        for el0, el1 in zip(v2.tolist(), v3.tolist()):
            self.assertEqual(el0, el1)


    def test_update_lines(self):
        n = 2
        x0 = np.zeros((3, n), dtype=np.float64)
        x1 = 10 * np.random.rand(3, n)

        self.map.update_lines(x0, x1)

        [x2, l2, v2] = self.map.get_voxels()

    def test_pass(self):
        pass

    def test_pickle(self):
        x0 = np.zeros((3, 10))
        x1 = np.random.uniform(-5, 5, (3, 10))
        self.map.update_lines(x0, x1)

        fd, path = tempfile.mkstemp()
        try:
            with os.fdopen(fd, 'wb') as f:
                pickle.dump(self.map, f)
            with open(path, 'rb') as f:
                loaded = pickle.load(f)
                self.assertEqual(self.map.voxel_size, loaded.voxel_size)
                self.assertEqual(self.map.free_update, loaded.free_update)
                self.assertEqual(self.map.hit_update, loaded.hit_update)
                self.assertEqual(self.map.occupied_threshold, loaded.occupied_threshold)
                x, _, val = self.map.get_voxels()
                x_loaded, _, val_loaded = loaded.get_voxels()
                self.assertEqual(x.ndim, x_loaded.ndim)
                self.assertEqual(x.min(), x_loaded.min())
                self.assertEqual(x.max(), x_loaded.max())
                self.assertEqual(val.ndim, val_loaded.ndim)
                self.assertEqual(val.min(), val_loaded.min())
                self.assertEqual(val.max(), val_loaded.max())
        finally:
            os.remove(path)

    # def test_trace_lines(self):
    #     self.map.clear()
    #     self.map.voxel_size = 0.5
    #     self.map.free_update = -1.0
    #     self.map.hit_update = 1.0
    #     n = 2
    #     x0 = np.zeros((3, n), dtype=np.float64)
    #     x1 = 10 * np.random.rand(3, n)
    #
    #     print('Tracing rays from:\n{},\nto:\n{}...'.format(x0, x1))
    #     min_val = -100.0
    #     max_val = 0.0
    #     [h, v] = self.map.trace_lines(x0, x1, min_val, max_val)
    #     print('Lines traced to:\n{}\nwith values\n{}.'.format(h, v))
    #
    # def test_trace_rays(self):
    #     self.map.clear()
    #     self.map.voxel_size = 0.5
    #     self.map.free_update = -1.0
    #     self.map.hit_update = 1.0
    #     n = 2
    #     x0 = np.zeros((3, n), dtype=np.float64)
    #     x1 = np.random.rand(3, n)
    #
    #     print('Tracing rays from:\n{},\nto:\n{}...'.format(x0, x1))
    #     max_range = 10.0
    #     min_val = -100.0
    #     max_val = 0.0
    #     [h, v] = self.map.trace_rays(x0, x1, max_range, min_val, max_val)
    #     print('Lines traced to:\n{}\nwith values\n{}.'.format(h, v))


if __name__ == '__main__':
    unittest.main()
