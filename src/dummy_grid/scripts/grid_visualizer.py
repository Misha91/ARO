#!/usr/bin/env python
from __future__ import division, print_function
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from dummy_grid.srv import DrawGrid, DrawGridRequest, DrawGridResponse
from exploration.utils import getRobotGridPosition
from matplotlib import pyplot as plt
from matplotlib import colors
import tf2_ros
import geometry_msgs.msg


def hexToRGB(hexcolor):
    return tuple(reversed([((c >> (i * 8)) & 0xff) / 255.0 for i, c in enumerate([hexcolor] * 3)]))


class GridVisualizer():

    def __init__(self):
        rospy.init_node("grid_visualizer")

        self.gridReady = False  # was the grid at least once received
        self.robotCircle = plt.Circle((-1, -1), 0.5, color=hexToRGB(0xf4a142))

        self.gridSubscriber = rospy.Subscriber('occupancy', OccupancyGrid, self.grid_cb)

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

    def drawGrid(self, request):

        enhGrid = self.grid.copy()
        ptList = np.array([(p.x, p.y, p.value) for p in request.points])
        if len(ptList) > 0:
            enhGrid[(ptList[:, 1], ptList[:, 0])] = ptList[:, 2]
        
        self.axesImage.set_data(enhGrid)
        plt.draw()
        plt.pause(0.001)
        return DrawGridResponse()

    def extractGrid(self, msg):
        self.ncols, self.nrows = msg.info.width, msg.info.height
        self.gridResolution = msg.info.resolution
        self.gridOriginPos = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.gridInfo = msg.info
        self.grid = np.reshape(msg.data, (self.ncols, self.nrows))

        if self.gridReady:
            try:
                trans = self.tfBuffer.lookup_transform("map", "robot", rospy.Time(), rospy.Duration(0.1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Cannot get the robot position, ignoring.")
            else:
                robotX, robotY = getRobotGridPosition(trans, self.gridInfo)
                self.robotCircle.center = (robotX, robotY)
            
            plt.draw()
            plt.pause(0.001)
        
    def grid_cb(self, msg):
        self.extractGrid(msg)
        if not self.gridReady:
            plt.ion()
            self.fig, self.ax = plt.subplots()
            cmap = colors.ListedColormap([
                hexToRGB(0xa3bcc9),
                (1.0, 1.0, 1.0),
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.2, 0.2, 1.0),
                hexToRGB(0x989b3f)
            ])
            bounds = [-1.5, -0.5, 10 , 100.5, 101.5, 102.5, 103.5, 104.5]
            ticks = [-1, 5, 50, 101, 102, 103, 104]
            bnorm = colors.BoundaryNorm(bounds, cmap.N)
            self.axesImage = self.ax.imshow(self.grid, cmap=cmap, norm=bnorm)
            colorbar = plt.colorbar(self.axesImage, ticks=ticks)
            colorbar.set_ticklabels(["unknown", "empty", "occupied", "start", "end", "path", "frontier"])

            # draw gridlines
            self.ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=0.1)
            self.ax.set_xticks(np.arange(-.5, self.ncols, 1))
            self.ax.set_yticks(np.arange(-.5, self.nrows, 1))
            self.ax.tick_params(labelleft="off", labelbottom="off", direction="in", length=0.0)

            self.ax.add_artist(self.robotCircle)

            # Create services
            self.drawGridSservice = rospy.Service('draw_grid', DrawGrid, self.drawGrid)
            self.gridReady = True
            plt.show(block=True)


if __name__ == "__main__":
    gv = GridVisualizer()
    rospy.spin()
