#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose2D
import numpy as np
import rospkg
import os
import tf.transformations as tft


class GridPublisher():

    def __init__(self):
        rospy.init_node("grid_publisher")

        gridPath = rospy.get_param("~grid_path", "data/grid_0.npy")
        self.offsetX = float(rospy.get_param("~x", 0.0))
        self.offsetY = float(rospy.get_param("~y", 0.0))
        self.theta = float(rospy.get_param("~theta", 0.0))
        self.resolution = float(rospy.get_param("~resolution", 0.1))
        self.frameID = rospy.get_param("~frame", "map")

        rospack = rospkg.RosPack()
        packagePath = rospack.get_path("dummy_grid")

        self.grid = np.load(os.path.join(packagePath, gridPath))
        self.makeMessage()

        self.rate = rospy.Rate(1)

        self.gridPublisher = rospy.Publisher('occupancy', OccupancyGrid, queue_size=3)

    def makeMessage(self):
        self.grid = np.flipud(self.grid)  # this is needed because of how the grid is oriented in NP and ROS
        self.msg = OccupancyGrid()
        self.msg.header.frame_id = self.frameID
        self.msg.info.resolution = self.resolution

        rot = tft.quaternion_about_axis(self.theta, (0, 0, 1))
        self.msg.info.origin.position.x = self.offsetX
        self.msg.info.origin.position.y = self.offsetY
        self.msg.info.origin.orientation.x, self.msg.info.origin.orientation.y, self.msg.info.origin.orientation.z, self.msg.info.origin.orientation.w = rot

        self.msg.info.height, self.msg.info.width = self.grid.shape
        self.msg.data = self.grid.astype(np.int8).flatten().tolist()

    def publish(self):
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            self.msg.header.stamp = stamp
            self.msg.info.map_load_time = stamp
            self.gridPublisher.publish(self.msg)
            self.rate.sleep()

if __name__ == "__main__":
    gp = GridPublisher()
    gp.publish()

