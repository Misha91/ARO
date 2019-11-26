#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import rospy
import numpy as np
from scipy.ndimage import morphology
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Pose2D
from exploration.srv import PlanPath, PlanPathRequest, PlanPathResponse
import tf2_ros
import math
import geometry_msgs.msg
import utils
from geometry_msgs.msg import TransformStamped, Transform
from geometry_msgs.msg import Vector3
from Queue import PriorityQueue
import matplotlib.pyplot as plt
"""
Here are imports that you are most likely will need. However, you may wish to remove or add your own import.
"""


class PathPlanner():

    def __init__(self, toPlot=False):
        rospy.init_node("path_planner")
        self.res = 0
        #self.tmp = 0
        #self.lookAroundStepsTmp = 0
        self.grid = []
        self.gridInfo = 0
        self.toPlot = toPlot
        # Get some useful parameters
        self.robotDiameter = float(rospy.get_param("~robot_diameter", 0.3))
        self.occupancyThreshold = int(rospy.get_param("/occupancy_threshold", 10))
        self.lookAroundSteps = 0
        # Helper variable to determine if grid was received at least once
        self.gridReady = False
        self.gridUpdated = False
        self.threshould = 0
        self.Update = True
        if (self.toPlot): plt.ion()
        self.tfBuffer = tf2_ros.Buffer()

        listener = tf2_ros.TransformListener(self.tfBuffer)


        self.gridSubscriber = rospy.Subscriber('occupancy', OccupancyGrid, self.grid_cb)

    def planPath(self, request):
        if (self.toPlot): plt.clf()
        self.Update = True
        while(not self.gridUpdated): pass
        self.gridUpdated = False
        """ Plan and return path from the robot position to the requested goal """
        # Get the position of the goal (real-world)
        goalPosition = np.array([request.goal.y, request.goal.x], dtype=np.float)
        tmp = TransformStamped()
        tmp.transform.translation.x = goalPosition[1]
        tmp.transform.translation.y = goalPosition[0]
        gridG = utils.getRobotGridPosition(tmp, self.gridInfo)
        posR, gridR = self.getRobotCoordinates()
        print(gridG, gridR, goalPosition, posR)

        path = []
        path = self.aStar(gridR, gridG)
        print(path)
        response = PlanPathResponse()
        real_path = [Pose2D(pos[0], pos[1], 0) for pos in [utils.gridToMapCoordinates(waypoint, self.gridInfo) for waypoint in path]]
        #if (len(path)):
        #    if (math.sqrt((posR.x - path[0][0])**2 + (posR.y - path[0][1])**2) > 0.55): response = PlanPathResponse()
        response = PlanPathResponse(real_path)
        #print(response)
        return response

    def getRobotCoordinates(self):
        """ Get the current robot position in the grid """
        try:
            #print("HEREHEREHERE")
            pose = rospy.wait_for_message("/odom", Odometry)
            #print(pose.pose.pose.position)
            pose = pose.pose.pose.position
            #trans = self.tfBuffer.lookup_transform("map", "odom", rospy.Time(), rospy.Duration(0.5))
            #trans = self.tfBuffer.lookup_transform("map", "robot", rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot get the robot position!")
            self.robotPosition = None
        else:
            #pos = np.array((trans.transform.translation.x, trans.transform.translation.y)).reshape(2,)
            #pos = np.array((pose.x, pose.y )).reshape(2,)
            tmp = TransformStamped()
            tmp.transform.translation.x = pose.x
            tmp.transform.translation.y = pose.y
            gridPos = utils.getRobotGridPosition(tmp, self.gridInfo)
            #print()
            return pose, gridPos
            #print(self.robotPosition)

    def heuristic(self, pos1, pos2):
        #print("Heur: ", pos1, pos2)
        return (abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]))

    def plotPlan(self, ind):
        plt.plot(ind[0], ind[1], 'bo')
        plt.pause(0.00000000000001)

    def aStar(self, start, goal):
        visited = []
        goal = [int(goal[0]), int(goal[1])]
        start = [int(start[0]), int(start[1])]
        frontier = PriorityQueue()
        path = []
        bestPathSoFar = [9999999, []]
        found = False
        mult = 75
        print(goal)
        #if (self.gridToInd(start) == False):
        #    print("Robot is out of map!")
        #    return None
        H = self.heuristic(start, goal)
        frontier.put((H, 0, start, [start]))

        print("H:", H, "*", mult, "=", H*mult)
        #if (self.gridToInd(goal) == False):
        #    print("Goal is out of map!")
        #    return None
        #goalInd = self.tmp
        dist = [[1,0],[-1,0],[0,1],[0,-1],[1,1],[-1,1],[1,-1],[-1,-1]]
        print(self.width*self.height)
        print(self.grid[start[0]][start[1]])
        print(self.grid[goal[0]][goal[1]])
        H = H * mult
        while not frontier.empty():
        #for i in range(0,10):
            current = frontier.get()
            #print("New iter #")
            #print(current)
            #print(frontier.qsize())
            #print(current[2], goal)
            if current[2] == goal:
                path = current[3]
                break
            if (current[2] in visited): continue
            else: visited.append(current[2])

            if (len(visited) > H): break

            for k in range(len(dist)):
                try:
                    #print("Cons: ", current[2][0] + dist[k][0], current[2][1] + dist[k][1])
                    if ([current[2][0] + dist[k][0], current[2][1] + dist[k][1]] == goal):
                        print("GOAL IS FOUND!")
                        found = True
                        cost = current[1] + 1
                        if (k >= 4): cost += 0.41
                        #ind_cp =
                        path_cp = current[3][:]
                        path_cp.append(goal)
                        frontier = PriorityQueue()
                        frontier.put((priority, cost, goal, path_cp))
                    else:
                        #print(current[2][0] + dist[k][0], current[2][1] + dist[k][1], dist[k], self.grid[current[2][0] + dist[k][0]][current[2][1] + dist[k][1]])
                        if (not found and ((self.grid[current[2][0] + dist[k][0]][current[2][1] + dist[k][1]] < 26 and self.grid[current[2][0] + dist[k][0]][current[2][1] + dist[k][1]] >= 0) or len(current[3]) < self.threshould)):
                            ind = [current[2][0] + dist[k][0], current[2][1] + dist[k][1]]

                            if (ind not in visited):

                                cost = current[1] + 1
                                if (k >= 4): cost += 0.41
                                #print("HERE")
                                heur = self.heuristic(ind, goal)
                                priority = cost + heur
                                path_cp = current[3][:]

                                path_cp.append(ind)
                                frontier.put((priority, cost, ind, path_cp))
                                #print("ADDED: ", priority, cost, ind, path_cp)
                                if bestPathSoFar[0] > heur:
                                    bestPathSoFar = [heur, path_cp]
                                #visited.append(ind)
                                if (self.toPlot): self.plotPlan(ind)
                except:
                    pass


        print(frontier.qsize(), len(visited))
        path_fin = []

        #    else:
        #        print("WTF IS GOING ON???")
        if (len(path) == 0):
            print("CANNOT FIND A PATH!")
            print(bestPathSoFar)
            #return bestPathSoFar[1]
            if (bestPathSoFar[0] < 35 and len(bestPathSoFar[1])>12): #self.threshould+5
                path = bestPathSoFar[1]
        if (len(path) <= 10 or len(path)>75): path = []
        for ind2, n in enumerate(path):
        #    if (self.indToGrid(n)):
            #print(self.grid[n[0]][n[1]])
            print(ind2, n , self.grid[n[0]][n[1]])
            #path_fin.append(np.array((n[0],n[1]), dtype=float))
            if (ind2 < (self.threshould) or (self.grid[n[0]][n[1]] <= 26 and self.grid[n[0]][n[1]] >= 0)): path_fin.append(np.array((int(n[0]),int(n[1])), dtype=int))

        return path_fin


    def extractGrid(self, msg):
        # TODO: extract grid from msg.data and other usefull information #,footprint=np.ones((3,3)), , structure =np. ones ((1, 1))
        #self.grid = morphology.grey_dilation(msg.data, size=(3,3))
        #self.grid = msg.data
        self.res = msg.info.resolution
        self.width = msg.info.width
        self.gridInfo = msg.info
        self.height = msg.info.height
        self.origin = msg.info.origin
        self.position = msg.info.origin.position
        self.orientation = msg.info.origin.orientation
        #print(self.origin, self.position, self.orientation, self.res)
        if (self.lookAroundSteps == 0):
            self.lookAroundSteps = 2 * float(self.robotDiameter) / float(self.res)
            self.lookAroundSteps = int(round(self.lookAroundSteps, 3))
            print("lAS: ", self.lookAroundSteps)
            tmp = round(self.lookAroundSteps)
            if (tmp >= self.lookAroundSteps): self.lookAroundSteps = int(tmp)
            else: self.lookAroundSteps = int(tmp) + 1
            self.lookAroundStepsTmp = self.lookAroundSteps
        self.grid = np.reshape(msg.data, (self.height, self.width)).T
        #for j in range(self.height):
        #    str = ""
        #    for i in range(self.width):
        #        if (self.grid[i][j] >0): str += "#"
        #        else: str += " "
            #print(str)
        print("UPDATED")
        self.grid = (morphology.grey_dilation(self.grid, size=(self.lookAroundSteps,self.lookAroundSteps)))
        self.threshould = int(self.lookAroundSteps*0.65) - 1
        #for j in range(self.height):
        #    str = ""
        #    for i in range(self.width):
        #        if (self.grid[i][j] >0): str += "#"
        #        else: str += " "
            #print(str)
        #self.threshould = 3
        self.Update = False
        self.gridUpdated = True

    def grid_cb(self, msg):

        if (self.Update): self.extractGrid(msg)
        if not self.gridReady:
            # TODO: Do some initialization of necessary variables
            #theta = round(round(self.orientation.z, 3)*2,2)
            #print("theta: ", theta)
            #c, s = np.cos(theta), np.sin(theta)
            #self.R = np.array(((c,-s), (s, c)))
            #self.t = (np.array((self.position.x, self.position.y))).reshape(2,)

            # Create services
            self.plan_service = rospy.Service('plan_path', PlanPath, self.planPath)
            self.gridReady = True


if __name__ == "__main__":
    pp = PathPlanner()

    rospy.spin()
