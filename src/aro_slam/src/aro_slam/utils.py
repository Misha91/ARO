from __future__ import absolute_import, division, print_function
from geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.spatial import cKDTree
from std_msgs.msg import Header
from voxel_map import VoxelMap
from transforms3d.quaternions import mat2quat
from sensor_msgs.point_cloud2 import create_cloud_xyz32, read_points


def slots(msg):
    """Return message attributes (slots) as list."""
    return [getattr(msg, var) for var in msg.__slots__]


def array(msg):
    """Return message attributes (slots) as array."""
    return np.array(slots(msg))


def col(arr):
    """Convert array to column vector."""
    return arr.reshape((arr.size, 1))


def transform_msg(R, t, stamp, frame, child_frame):
    """Create TF message from rotation matrxi and translation vector."""
    # Convert rotation matrix to 'xyzw' quaternion.
    q = mat2quat(R)[[1, 2, 3, 0]]  # to
    # Compose and return ROS message.
    msg = TransformStamped(Header(None, stamp, frame),
                           child_frame,
                           Transform(Vector3(*t.ravel()),
                                     Quaternion(*q)))
    return msg


def points_3d(x):
    """Pad with zeros up to tree rows."""
    assert x.shape[0] <= 3
    x = np.concatenate((x, np.zeros((3 - x.shape[0], x.shape[1]))))
    return x


def cloud_msg(x, stamp, frame):
    """Create cloud message, fill missing dimension(s) with zeros."""
    x = points_3d(x)
    # if x.shape[0] == 2:
    # x = np.concatenate((x, np.zeros((3 - x.shape[0], x.shape[1]))))
    msg = create_cloud_xyz32(Header(None, stamp, frame), x.T)
    return msg


def points_from_msg(msg, fields=('x', 'y')):
    x = read_points(msg, field_names=fields, skip_nans=True)
    x = np.array(list(x), dtype=np.float64).T
    return x


def filter_grid(x, grid_res):
    """Select random point within each cell. Order is not preserved."""
    # Convert points in cols to rows, and shuffle the rows.
    x = np.array(x.T, copy=True)
    np.random.shuffle(x)
    # Get integer cell indices, as tuples.
    idx = np.floor(x / grid_res)
    idx = [tuple(i) for i in idx]
    # Dict keeps the last value for each key, which is random due to shuffle.
    x = dict(zip(idx, x))
    # Concatenate and convert rows to cols.
    x = np.stack(x.values()).T
    return x


class PointMap(object):
    def __init__(self, grid_res=None, max_size=None):
        self.points = None
        self.index = None
        self.grid_res = grid_res
        self.max_size = max_size

    def size(self):
        if self.points is None:
            return 0
        return self.points.shape[1]

    def empty(self):
        return self.size() == 0
    
    def update_index(self):
        self.index = cKDTree(self.points.T)

    def update(self, x):
        if self.points is None:
            self.points = x
        else:
            self.points = np.concatenate((self.points, x), axis=1)
        if self.grid_res is not None:
            self.points = filter_grid(self.points, self.grid_res)
        if self.max_size is not None and self.size() > self.max_size:
            keep = np.random.choice(self.size(), self.max_size, replace=False)
            self.points = self.points[:, keep]
        self.update_index()

def logistic(x):
    return 1. / (1. + np.exp(-x))


class OccupancyMap(object):
    def __init__(self, frame_id, resolution=0.1):
        self.voxel_map = VoxelMap(resolution, -1.0, 2.0, 0.0)
        self.min = -10.0
        self.max = 10.0
        self.msg = OccupancyGrid()
        self.msg.header.frame_id = frame_id
        self.msg.info.resolution = resolution
        # Initially, set the grid origin to identity.
        self.msg.info.origin.orientation.w = 1.0

    def map_to_grid(self, x):
        """Transform points from map coordinates to grid."""
        # TODO: Handle orientation too.
        x = x - col(array(self.msg.info.origin.position))
        return x

    def grid_to_map(self, x):
        """Transform points from grid coordinates to map."""
        # TODO: Handle orientation too.
        x = x + col(array(self.msg.info.origin.position))
        return x

    def fit_grid(self):
        """Accommodate the grid to contain all points."""
        # Update grid origin so that all coordinates are non-negative.
        x, _, v = self.voxel_map.get_voxels()
        x = x[:2]  # Only x,y used in 2D grid.
        x_min = x.min(axis=1) - self.voxel_map.voxel_size / 2.
        x_max = x.max(axis=1) + self.voxel_map.voxel_size / 2.
        nx = np.round((x_max - x_min) / self.msg.info.resolution).astype(np.int)
        self.msg.info.origin.position = Point(x_min[0], x_min[1], 0.0)
        self.msg.info.width, self.msg.info.height = nx

    def grid_voxels(self):
        """Return voxel coordinates corresponding to the current grid."""
        i, j = np.meshgrid(np.arange(self.msg.info.width),
                           np.arange(self.msg.info.height),
                           indexing='xy')
        x = np.stack((i.ravel(), j.ravel(), np.zeros_like(i).ravel()))
        x = (x + 0.5) * self.msg.info.resolution
        return x

    def to_msg(self):
        """Return as grid message. (Update grid parameters as needed.)"""
        self.fit_grid()
        x = self.grid_voxels()
        x = self.grid_to_map(x)
        x[2, :] = self.voxel_map.voxel_size / 2.0
        l = np.zeros((x.shape[1],))
        v = self.voxel_map.get_voxels(x, l)
        v = 100. * logistic(v)
        v[np.isnan(v)] = -1.
        self.msg.data = v.astype(int).tolist()
        return self.msg
    
    def voxel_map_points(self, x):
        x = points_3d(x.copy())
        x[2, :] = self.voxel_map.voxel_size / 2.0
        return x

    def update(self, x, y, stamp):
        """Update internal occupancy map."""
        x = self.voxel_map_points(x)
        y = self.voxel_map_points(y)
        if x.shape[1] == 1:
            x = np.broadcast_to(x, y.shape)
        elif y.shape[1] == 1:
            y = np.broadcast_to(y, x.shape)
        self.voxel_map.update_lines(x, y)
        self.clip_values()
        self.msg.header.stamp = stamp
        self.msg.info.map_load_time = stamp
    
    def occupied(self, x):
        x = self.voxel_map_points(x)
        l = np.zeros((x.shape[1],))
        v = self.voxel_map.get_voxels(x, l)
        occupied = v > self.voxel_map.occupied_threshold
        return occupied
    
    def clip_values(self):
        x, l, v = self.voxel_map.get_voxels()
        v = np.clip(v, self.min, self.max)
        self.voxel_map.set_voxels(x, l, v)
