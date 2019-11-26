#!/usr/bin/env python
from __future__ import division, print_function
import rospy
import numpy as np
from scipy.ndimage import morphology
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Pose2D
from exploration.srv import AnyFrontiersLeft, AnyFrontiersLeftResponse, GenerateFrontier, GenerateFrontierResponse
import tf2_ros
from collections import Counter
import math
import geometry_msgs.msg
import matplotlib.pyplot as plt
import utils
from geometry_msgs.msg import TransformStamped, Transform
from geometry_msgs.msg import Vector3
from Queue import *
"""
Here are imports that you are most likely will need. However, you may wish to remove or add your own import.
"""


class FrontierExplorer():

    def __init__(self):
        # Initialize the node
        rospy.init_node("frontier_explorer")
        self.res = 0
        self.tmp = 0
        self.grid = []
        self.grid2 = []
        self.reallyUpdated = False
        self.gridInfo = []
        self.frontiersList = []
        # Get some useful parameters
        self.robotDiameter = float(rospy.get_param("~robot_diameter", 0.2))
        self.occupancyThreshold = int(rospy.get_param("/occupancy_threshold", 10))
        self.lookAroundSteps = 0
        # Helper variable to determine if grid was received at least once
        self.gridReady = False
        self.Update = True
        # You may wish to listen to the transformations of the robot
        self.tfBuffer = tf2_ros.Buffer()
        # Use the tfBuffer to obtain transformation as needed
        tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.R = np.empty((2,2))
        self.t = np.empty((2,))
        # Subscribe to grid
        self.gridSubscriber = rospy.Subscriber('occupancy', OccupancyGrid, self.grid_cb)

        # TODO: you may wish to do initialization of other variables

    def toGrid(self, pos):
        robotPos = ((self.R.T).dot(pos-self.t))/ self.res  # TODO: transform the robot coordinates from real-world (in meters) into grid
        robotPos[0] = round(robotPos[0])
        robotPos[1] = round(robotPos[1])
        robotPos = np.array((robotPos), dtype=int)
        return robotPos

    def toCoord(self, pos):
        ret = (self.R.dot(pos*self.res) + self.t)
        #print(ret)
        #if len(ret)>=2:
        #    ret[0] = round(ret[0], 3)
        #    ret[1] = round(ret[1], 3)
        return ret

    def isFrontier(self, ind):
        if (self.grid[ind] != -1): return False
        if (ind%self.width == 0): dist = [1, self.width, self.width + 1, -self.width, -self.width + 1]
        elif (ind%self.width == self.width-1): dist = [-1, self.width, self.width - 1, -self.width, -self.width - 1]
        else: dist = [1, self.width, -1, -self.width, self.width - 1, self.width + 1, -self.width + 1, -self.width - 1]
        #toReturn = True
        for k in dist:
            if self.indToGrid(ind+k):
                if self.grid[ind+k] == 0: return True
        return False

    def WFD(self, robotInd):
        print("Searching started....")
        qM = Queue()
        mol = []
        mcl = []
        fol = []
        fcl = []
        status = {}
        for i in range (0, self.width * self.height):
            status[i] = "non"
        frontiers = []
        qM.put(robotInd)
        #mol.append(robotInd)
        status[robotInd] = "mol"
        dist = [1, self.width, -1, -self.width, self.width-1, self.width + 1, -self.width + 1, -self.width - 1]

        while not qM.empty():
            #print(mol, '\n', mcl, '\n', fol, '\n', fcl, '\n', frontiers, '\n', qM.qsize())
            p = qM.get()
            if (status[p] == "mcl"): continue

            if (self.indToGrid(p) and self.isFrontier(p)):
                #print(p)
                qF = Queue()
                nf = []
                qF.put(p)
                status[p] = "fol"
                while not qF.empty():
                    q = qF.get()
                    if (status[q] == "mcl" or status[q] == "fcl"): continue
                    if (self.indToGrid(q) and self.isFrontier(q)):
                        nf.append(q)
                        for k in dist:
                            if (self.indToGrid(q+k) and status[q+k] != "fol" and status[q+k] != "fcl" and status[q+k] != "mcl"):
                                qF.put((q+k))
                                status[q+k] = "fol"
                                #fol.append((q+k))
                    status[q] = "fcl"
                if len(nf)>= 1:
                    for i in nf:
                        print(len(nf), "frontier points added")
                        frontiers.append(i)
                        status[i] = "mcl"

                    #return frontiers
            for k in dist:
                if (self.indToGrid(p+k)):
                    if (status[p+k] != "mol" and status[p+k] != "mcl"):
                        for j in dist:
                            if (self.indToGrid(p+k+j) and (self.grid[p+k+j] == 0)):
                                qM.put((p+k))
                                status[p+k] = "mol"
                                break
            status[p] = "mcl"
        #print(frontiers)
        #self.plotWFD(frontiers)
        return frontiers

    def computeWFD(self):
        """ Run the Wavefront detector """
        frontiers = []
        # TODO:
        # TODO: First, you should try to obtain the robots coordinates

        robotC = self.getRobotCoordinates()

        robotInd = 0
        if (self.gridToInd(robotC)): robotInd = self.tmp


        # TODO: Then, copy the occupancy grid into some temporary variable and inflate the obstacles
        # TODO: Run the WFD algorithm - see the presentation slides for details on how to implement it

        frontiers = self.WFD(robotInd)
        #frontiers.sort()

        return frontiers

    def isNeighbor(self, arr, b):
        #dist = [1, 50, 49, 51, 50]
        dist = [1, self.width, self.width-1, self.width+1, -1, -self.width, -self.width-1, -self.width+1]
        for a in arr:
            diff = abs(a-b)
            for k in dist:
                if k == diff: return True
        return False

    def degroupF(self, frontiers):

        tmp = []
        toRet = []
        #if (type(frontiers) == list and len(frontiers) >0):
            #print(type(frontiers))
        tmp.append(frontiers[0])
        for n in range(1, len(frontiers)):

            if self.isNeighbor(tmp, frontiers[n]):
                #print(frontiers[n], "added to tmp")
                tmp.append(frontiers[n])
            else:
                toRet.append(tmp)
                tmp = []
                tmp.append(frontiers[n])
                #print(frontiers[n], "added to new")
        if (len(tmp)): toRet.append(tmp)
        print("Frontier groups:")
        print(len(toRet))
        #toRet2 = []
        #for g in toRet:
        #    if (len(g) >= 20):
        #        toRet2.append(g)
        #toRet = toRet2
        #print(len(toRet))
        cnt = []
        for i in toRet:
            xmax = 0
            ymax = 0
            xmin = 99999
            ymin = 99999
            for k in i:
                self.indToGrid(k)
                if xmax <= self.tmp[0]: xmax = self.tmp[0]
                if xmin >= self.tmp[0]: xmin = self.tmp[0]
                if ymax <= self.tmp[1]: ymax = self.tmp[1]
                if ymin >= self.tmp[1]: ymin = self.tmp[1]
            cnt.append(np.array(([int(round((xmax+xmin)/2)), int(round((ymax+ymin)/2))])))
        #print(cnt)
        points = []
        for g in toRet:
            for p in g:
                points.append(p)
        #self.plotWFD(points)
        return toRet, cnt

    def indToGrid(self, ind):
        x = ind%self.width
        y = ind//self.width
        if (x>=0 and x <= self.width - 1 and y >= 0 and y <= self.height - 1):
            self.tmp = np.array([int(x), int(y)])
            return True
        else: return False

    def gridToInd(self, pos):
        result = pos[1]*self.width + pos[0]
        if (result >= 0 and result <= ((self.width * self.height) - 1)):
            self.tmp = int(result)
            return True
        else: return False

    def plotWFD(self, frontier):
        x, y = [], []
        for i in frontier:
            if (self.indToGrid(i)):
                tmp = utils.gridToMapCoordinates(np.array(self.temp), self.gridInfo)
                x.append(tmp[0])
                y.append(tmp[1])
        plt.plot(x, y, 'o-')

        #ax.tick_params(width=10)
        plt.ion()
        #plt.clear()
        plt.show()

    def anyFrontiersLeft(self, request):
        """ Check if there are any frontiers left """
        # Run the WFD
        frontiers = self.computeWFD()
        # Return True if there are any frontiers, False otherwise
        return AnyFrontiersLeftResponse(any_frontiers_left=bool(len(frontiers) > 0))


    def heuristic(self, pos, target):
        return (math.sqrt((pos[0] - target[0])**2 + (pos[1] - target[1])**2))

    def getRandomFrontier(self, request):
        self.Update = True
        """ Return random frontier """
        # TODO
        frontiers = self.computeWFD()
        frontier = np.random.choice(frontiers)
        #print(frontier)
        frontierCenter = 0
        grp, cnt = self.degroupF(frontiers)
        for k in range(len(grp)):
            if frontier in grp[k]:
                frontierCenter = cnt[k]
        #print(frontierCenter)
        pos = utils.gridToMapCoordinates(np.array(frontierCenter), self.gridInfo)
        #  # TODO: compute center of the randomly drawn frontier here
        x, y = pos[0], pos[1]  # TODO: transform the coordinates from grid to real-world coordinates (in meters)
        response = GenerateFrontierResponse(Pose2D(x, y, 0.0))
        return response

    def getClosestFrontier(self, request):
        pos = self.getRobotCoordinates()
        self.reallyUpdated = False
        self.Update = True
        while (not self.reallyUpdated): pass
        self.reallyUpdated = False
        #print("HERE")
        if (len(self.frontiersList) == 0):
            try:
                print("FINDING NEW FRONTS")
                #self.Update = True
                """ Return frontier closest to the robot """
                # TODO
                frontiers = self.computeWFD()
                #print("here2")
                if (len(frontiers)>0):
                    grp, cnt = self.degroupF(frontiers)
                    for c in cnt:
                        print(utils.gridToMapCoordinates(np.array(c), self.gridInfo))

                    #print("here3")
                    #dist = [1, self.width, self.width-1, self.width+1, -1, -self.width, -self.width-1, -self.width+1]
                    """
                    for c in range(len(cnt)):
                        if (self.gridToInd(cnt[c])):
                            if self.grid[self.tmp] != 0:
                                ind = self.tmp
                                for d in dist:
                                    #print("IND+D: ", ind+d)
                                    if (self.indToGrid(ind+d) and self.grid[ind + d] == 0):
                                        cnt[c] = self.tmp
                                        break
                    """
                    self.frontiersList = cnt
                    print("FRONTS FOUND: ", len(self.frontiersList))
                else: self.frontiersList = []
                #print("here4")
            except:
                self.frontiersList = []

        #else:
        min = 999
        minCnt = 0
        #print(cnt)
        response = GenerateFrontierResponse(Pose2D(-999, -999, 0.0))
        #    for n in cnt:
        #        tmp = self.toCoord(n)
        #        print(tmp)
        #        response = (GenerateFrontierResponse(Pose2D(tmp[0], tmp[1], 0.0)))
        if (len(self.frontiersList)):
            toRemove = np.array(())
            print("Currnently have:", len(self.frontiersList))
            tmp2 = []
            for n in self.frontiersList:
                if (not self.checkValidity(n)): continue
                else: tmp2.append(n)
                tmpVal = self.heuristic(pos, n)
                #print(pos, n, tmpVal)
                if tmpVal <= min:
                    minCnt = utils.gridToMapCoordinates(np.array(n), self.gridInfo)
                    min = tmpVal
                    toRemove = n
            print(min, len(self.frontiersList))
            #if (min > 50 and len(self.frontiersList) <= 3):
                #print("FRONT RESTART! ", min, len(self.frontiersList))
                #self.frontiersList = []
                #self.getRandomFrontier(request)
                #return
            tmp = []
            for n in tmp2:
                if n[0] != toRemove[0] and n[1] != toRemove[1]:
                    tmp.append(n)
            self.frontiersList = tmp
            print("And now... :", len(self.frontiersList))
            print(self.frontiersList)
            print("minCnt", minCnt)
            try:
                x, y = round(minCnt[0],5), round(minCnt[1],5)  # TODO: compute the index of the best frontier
                print("CF: ", x, y)
                response = GenerateFrontierResponse(Pose2D(x, y, 0.0))
                return response
            except:
                response = self.getClosestFrontier(request)
                return response
        return response

    def checkValidity(self, p):

        dist = [[1,0],[-1,0],[0,1],[0,-1],[1,1],[-1,1],[1,-1],[-1,-1]]
        c = Counter()
        for k in range(0,int(5/(self.res) + 1)): #1
            for d in dist:
                try:
                    if self.grid2[p[0]+d[0]][p[1]+d[1]] == -1: c["unknown"] += 1
                    elif self.grid2[p[0]+d[0]][p[1]+d[1]] == 0: c["free"] += 1
                    else: c["obstacle"] += 1
                except:
                    pass
        print(p, utils.gridToMapCoordinates(np.array(p), self.gridInfo), c["unknown"], c["obstacle"], float(sum(c.values())))
        if (c["free"]/float(sum(c.values())) > 0.8 or c["unknown"]/float(sum(c.values())) <= 0.275 or (c["unknown"] + c["obstacle"])/float(sum(c.values()))  >= 0.95): return False #0.8, 0.25, 0.95
        else: return True

        #print(self.grid2[p[0]][p[1]])
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
            return gridPos
            #print(self.robotPosition)

    def extractGrid(self, msg):
        # TODO: extract grid from msg.data and other usefull information
        self.grid = msg.data

        self.gridInfo = msg.info
        self.res = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin = msg.info.origin
        self.position = msg.info.origin.position
        self.orientation = msg.info.origin.orientation
        self.grid2 = np.reshape(msg.data, (self.height, self.width)).T
        self.Update = False
        self.reallyUpdated = True
        print("MAP UPDATED")

    def grid_cb(self, msg):
        if (self.Update): self.extractGrid(msg)
        if not self.gridReady:
            # TODO: Do some initialization of necessary variables
            theta = round(round(self.orientation.z, 3)*2,2)
            c, s = np.cos(theta), np.sin(theta)
            self.R = np.array(((c,-s), (s, c)))
            self.t = (np.array((self.position.x, self.position.y))).reshape(2,)
            self.lookAroundSteps = 0.75 * self.robotDiameter / self.res
            tmp = round(self.lookAroundSteps)
            if (tmp >= self.lookAroundSteps): self.lookAroundSteps = int(tmp)
            else: self.lookAroundSteps = int(tmp) + 1
            # Create services


            self.afl_service = rospy.Service('any_frontiers_left', AnyFrontiersLeft, self.anyFrontiersLeft)
            self.grf_service = rospy.Service('get_random_frontier', GenerateFrontier, self.getRandomFrontier)
            self.gcf_service = rospy.Service('get_closest_frontier', GenerateFrontier, self.getClosestFrontier)
            self.gridReady = True


if __name__ == "__main__":
    fe = FrontierExplorer()

    rospy.spin()
