#!/usr/bin/env python
"""
Simultaneous localization and mapping (SLAM) based on iterative closest point
(ICP). The node operates in 2D. Z coordinate is discarding when processing
point clouds.
In brief, the node receives point clouds which it registers with incrementally
built map to estimate transformation between odom and map frame.
"""

from __future__ import absolute_import, division, print_function
from laser_geometry.laser_geometry import LaserProjection
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
import numpy as np
import rospy
import math
import sys
from sensor_msgs.msg import LaserScan, PointCloud2
from timeit import default_timer as timer
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
# Needed for tf2_ros.Buffer.transform(cloud)
import geometry_msgs.msg
from std_msgs.msg import Int32, String
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import Pose2D, Pose, Point, Vector3, PointStamped, Twist
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from exploration.srv import PlanPath,  PlanPathRequest, PlanPathResponse, AnyFrontiersLeft, AnyFrontiersLeftRequest, AnyFrontiersLeftResponse, GenerateFrontier, GenerateFrontierResponse
import time
from scipy import spatial
from robot_coordination.msg import Waypoint
from robot_coordination.srv import AddPath
from robot_coordination.srv import StartMovement
from robot_coordination.srv import StopMovement
from random import randint

class PathFollowing:
    def __init__(self):
        self.state_sub = rospy.Subscriber('waypoints_ahead', Int32, self.callback_state)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.waypoints_ahead = 0
        self.waypoints_ahead_updated = False

    def callback_state(self, msg):
        self.waypoints_ahead = msg.data
        self.waypoints_ahead_updated = True

    def wait_for_service(self, srv_name):
        """Wait for a service called srv_name."""
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(srv_name, 5)
                return True
            except rospy.ROSException:
                rospy.logwarn('Could not connect to service {}/{}, trying again'.format(
                    self.server_ns, srv_name))
            except (rospy.ROSInterruptException, KeyboardInterrupt):
                return False

    def start_movement(self, backwards=False):
        """Start the robot motion by calling the service 'start_movement'"""
        srv_name = '/start_movement'
        if not self.wait_for_service(srv_name):
            return False
        try:
            start_movement_srv = rospy.ServiceProxy(srv_name, StartMovement)
            reply = start_movement_srv(backwards)
            return reply.ack
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
        self.waypoints_ahead_updated = False
        return False

    def stop_movement(self):
        """Stop the robot motion by calling the service 'stop_movement'"""
        srv_name = '/stop_movement'
        if not self.wait_for_service(srv_name):
            return False
        try:
            stop_movement_srv = rospy.ServiceProxy(srv_name, StopMovement)
            reply = stop_movement_srv()
            return reply.ack
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
        return False

    def add_path(self, waypoint_list):
        """Add a path with data from trajectory"""
        srv_name = '/add_path'
        if not self.wait_for_service(srv_name):
            return False
        try:
            add_path_srv = rospy.ServiceProxy(srv_name, AddPath)
            reply = add_path_srv(waypoint_list)
            return reply.result
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
        return False

    def path_finished(self):
        return (self.waypoints_ahead <= 0) and self.waypoints_ahead_updated


class routine(object):

    def __init__(self):
        print('init start')
        rospy.init_node('routine')
        self.is_bb = rospy.Subscriber('barbie_point', PointStamped, self.goto_bb, queue_size=10)
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.barbieFound = False
        self.needPause = False
        self.barbiePoint = []
        self.toGotoBarbie = []
        self.frontiers = []
        self.lastPointInPath = []
        self.pointsVisited = np.array(([[0, 0]]))
        self.closF_x = 0
        self.closF_y = 0
        self.newTwist = 1.5
        self.barbieStopTwist = False
        self.twistCntr = 0
        self.lastPath = []
        self.visitedFronts = []
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.barbiePointPublisher = rospy.Publisher("barbie_position_final", String, queue_size=10)
        self.pathPublisher = rospy.Publisher("path", MarkerArray, queue_size=10)
        self.frontPublisher = rospy.Publisher("frontier", Marker, queue_size=10)
        self.velocity_publisher = rospy.Publisher('cmd_vel_mux/safety_controller', Twist, queue_size=10)
        print('init end')

    def getRobotCoordinates(self):
        """ Get the current robot position in the grid """
        try:
            pose = rospy.wait_for_message("/odom", Odometry)
            print(pose.pose.pose.position)
            pose = pose.pose.pose.position
            #trans = self.tfBuffer.lookup_transform("map", "odom", rospy.Time(), rospy.Duration(0.5))
            #trans = self.tfBuffer.lookup_transform("map", "robot", rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot get the robot position!")
            self.robotPosition = None
        else:
            #pos = np.array((trans.transform.translation.x, trans.transform.translation.y)).reshape(2,)
            pos = np.array((pose.x, pose.y)).reshape(2,)
            print(pos)
            return pos

            #print(self.robotPosition)

    def getRobotCoordinates2(self):
        """ Get the current robot position in the grid """
        try:
            #pose = rospy.wait_for_message("/odom", Odometry)
            #print(pose.pose.pose.position)
            #pose = pose.pose.pose.position
            trans = self.tfBuffer.lookup_transform("map", "barbie_point", rospy.Time(), rospy.Duration(0.5))
            #trans = self.tfBuffer.lookup_transform("map", "robot", rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot get the robot position!")
            self.robotPosition = None
        else:
            pos = np.array((trans.transform.translation.x, trans.transform.translation.y)).reshape(2,)
            #pos = np.array((pose.x, pose.y)).reshape(2,)
            print(pos)
            return pos
            #print(self.robotPosition)

    def makeTurnAround(self, tmp=True):
        #if (tmp): return
        if (self.twistCntr == 0 or self.twistCntr > 5):
            print("turn around start")
            vel_msg = Twist()
            speed = 4
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.newTwist
            pose = rospy.wait_for_message("/framed", PointStamped)
            for x in xrange(1, 13):

                self.velocity_publisher.publish(vel_msg)
                rospy.sleep(0.5)
                if (self.twistCntr != 0):
                    #pass
                    pose = rospy.wait_for_message("/framed", PointStamped)
                #if (self.barbieStopTwist): break

            #if (self.barbieStopTwist): rospy.sleep(10)
            #lse: self.barbieStopTwist = False

            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            print("turn around end")
            self.newTwist = -self.newTwist
        self.twistCntr += 1

    def goto_bb(self, msg):
        self.barbieStopTwist = True
        msg = self.tfBuffer.transform(msg, self.map_frame)
        print("BARBIE HERE (RAW): ", msg.point.x, msg.point.y)
        msg.point.x = round(abs(msg.point.x) + 0.5, 2)
        msg.point.y = round(abs(msg.point.y) + 0.5, 2)
        tmpString = str("Barbie X: " + str(msg.point.y) + "m, Y:" + str(msg.point.x) +"m")
        self.barbiePointPublisher.publish(tmpString)
        print("BARBIE HERE: ", msg.point.x, msg.point.y)
        #if not self.barbieFound:
        if (False):
            print("BARBIE!")

            msg = self.tfBuffer.transform(msg, self.map_frame)
            #msg = self.tfBuffer.transform(msg, self.robot_frame)
            while(self.needPause): pass
            #print(msg)
            pos = self.getRobotCoordinates()
            #print(self.getRobotCoordinates2())
            bx = msg.point.x
            by = msg.point.y
            dist = math.sqrt((pos[0]- bx)**2 + (pos[1]-by)**2)
            distToBarbie = 0.5
            #print(pos)
            #dx = (msg.point.x - pos[0])
            #dy = (msg.point.y - pos[1])
            v = [[bx - distToBarbie, by - distToBarbie] , [bx - distToBarbie, by + distToBarbie], \
            [bx + distToBarbie, by + distToBarbie], [bx + distToBarbie, by + distToBarbie], \
            [bx, by + distToBarbie], [bx, by - distToBarbie], \
            [bx + distToBarbie, by], [bx - distToBarbie, by]]

            min_dist = dist + 10
            minV = []

            for i in range(len(v)):
                distTmp = math.sqrt((pos[0]-v[i][1])**2 + (pos[1]-v[i][0])**2)
                if distTmp <= min_dist:
                    min_dist = distTmp
                    minV = v[i]

            print("MSG: ", pos, bx, by, minV, min_dist, dist, "END")
            self.barbieFound = True
            self.barbiePoint = msg.point
            #minV = [pos[0], pos[1]]
            self.toGotoBarbie = minV
            #rbt.getpath(x[0], x[1])
            #while (barbie_pos - robot_pos)< margin:
            #    self.getpath(barbie_point.point.x, barbie_point.point.y)
            #    self.create_trajectory()

            #self.result = rospy.Publisher('barbie_point', PointStamped, queue_size=1)
            #self.result.publish(barbie_point)


    def findFrontier(self):
        #rospy.init_node("front_tester")

        cmd = rospy.get_param("~cmd", "near")
        #publisher = rospy.Publisher("frontier", Marker, queue_size=10)
        print("finding...")


        if cmd.lower() == "near":
            rospy.wait_for_service("get_closest_frontier")
            caller = rospy.ServiceProxy("get_closest_frontier", GenerateFrontier)
            response = caller.call()
            print(response)

            #rospy.sleep(0.5)
            header = Header(frame_id="map")
            msg = Marker(header=header, pose=Pose(position=Point(response.goal_pose.x, response.goal_pose.y, 0)), id=np.random.randint(0, 1e9), type=Marker.CUBE, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 1, 0, 1), lifetime=rospy.Duration(0))
            #publisher.publish(msg)
            #rospy.loginfo(response)
            header = Header(frame_id="map")
            msg = Marker(header=header, pose=Pose(position=Point(response.goal_pose.x, response.goal_pose.y, 0)), id=np.random.randint(0, 1e9), type=Marker.CUBE, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 1, 0, 1), lifetime=rospy.Duration(0))
            self.frontPublisher.publish(msg)
            #rospy.loginfo(response)
            #self.frontPublisher

            #rospy.init_node("path_tester")
            #publisher = rospy.Publisher("path", MarkerArray, queue_size=10)
            #print(response.goal_pose.x)
            if (response.goal_pose.x == -999): return False
            self.closF_x = response.goal_pose.x
            self.closF_y = response.goal_pose.y
            self.frontiers.append([response.goal_pose.x, response.goal_pose.y])
            print("Frontier added: ", self.closF_x, ", ", self.closF_y )
            #if (len(response2)): return True
            #else: return False

        return True

    def getpathF(self):

        print('start')

        publisher = rospy.Publisher("path", MarkerArray, queue_size=10)

        xPos = rospy.get_param("~x", self.closF_x)
        yPos = rospy.get_param("~y", self.closF_y)

        rospy.wait_for_service("plan_path")

        caller = rospy.ServiceProxy("plan_path", PlanPath)


        request = PlanPathRequest(Pose2D(xPos, yPos, 0.0))
        response = caller.call(request)

        self.lastPath = []

        for p in response.path:
            self.lastPath.append([p.x, p.y])
            #print(p.x, p.y)
        print(self.lastPath)

    def getpath(self, x, y):

        print('start')



        xPos = rospy.get_param("~x", x)
        yPos = rospy.get_param("~y", y)
        print('0')
        rospy.wait_for_service("plan_path")
        print('00')
        caller = rospy.ServiceProxy("plan_path", PlanPath)


        request = PlanPathRequest(Pose2D(xPos, yPos, 0.0))
        response = caller.call(request)
        self.lastPath = []
        for p in response.path:
            self.lastPath.append([p.x, p.y])
        #rospy.sleep(0.5)
        header = Header(stamp=rospy.Time.now(), frame_id="map")
        msg = MarkerArray([Marker(header=header, pose=Pose(position=Point(p.x, p.y, 0)), id=np.random.randint(0, 1000), type=1, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 0.5, 1, 1)) for p in response.path])
        self.pathPublisher.publish(msg)
        print(self.lastPath)
        if (len(self.lastPath)): return True
        else: return False

    def create_trajectory(self):
        """ Create a trajectory in the odometry frame. """
        # x,y,time
        gtr = []
        seconds = 0.1
        for p in self.lastPath:
            self.pointsVisited = np.vstack((self.pointsVisited, [p[0], p[1]]))
            gtr.append([p[0], p[1], seconds])
            seconds += 0.01 #0.1
        self.lastPointInPath = [self.lastPath[-1][0], self.lastPath[-1][1]]
        waypoint_list = []
        for r in gtr:
            waypoint = Waypoint()
            waypoint.pose = Pose()
            waypoint.pose.position.x = r[0]
            waypoint.pose.position.y = r[1]
            waypoint.timepoint = rospy.Duration.from_sec(r[2])
            waypoint_list.append(waypoint)

        return waypoint_list





        #rospy.loginfo(response)

        #print(response)

        #rospy.sleep(0.5)
        #header = Header(stamp=rospy.Time.now(), frame_id="map")
        #msg = MarkerArray([Marker(header=header, pose=Pose(position=Point(p.x, p.y, 0)), id=np.random.randint(0, 1000),\
            #type=1, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 0.5, 1, 1)) for p in response.path])
        #publisher.publish(msg)


'''
        create_trajectory()
    def create_trajectory(self, path):
        """ Create a trajectory in the odometry frame. """
        # x,y,time


        gtr = [
            [0.0, 0.0, 0],
            [0.25, 0.25, 4],
            [0.50, 0.25, 8],
            [0.50, 0.50, 12],
            [0.50, 0.75, 16],
        ]
        waypoint_list = []
        for r in gtr:
            waypoint = Waypoint()
            waypoint.pose = Pose()
            waypoint.pose.position.x = r[0]
            waypoint.pose.position.y = r[1]
            waypoint.timepoint = rospy.Duration.from_sec(r[2])
            waypoint_list.append(waypoint)
        return waypoint_list
'''


def walkRoutine(waypoint_list):
    pf = PathFollowing()
    rate = rospy.Rate(10)

    #waypoint_list = []
    if not pf.add_path(waypoint_list):
        rospy.logerr('Could not add path, exiting')
        return
    rospy.loginfo('Path added')
    if not pf.start_movement(backwards=False):
        rospy.logerr('Could not start motion, exiting')
        return
    rospy.loginfo('Movement started')
    while not pf.path_finished() and not rospy.is_shutdown():
        rate.sleep()
    rospy.loginfo('Movement ended')
    if not pf.stop_movement():
        rospy.logerr('Could not stop motion, exiting')
        return
    rospy.loginfo('Movement stopped')
    if not pf.add_path([]):
        rospy.logerr('Could not clear path, exiting')
        return
    rospy.loginfo('Path cleared')


def main():
    T = True
    F = False
    movement = 0.75
    randomWalks = False
    frontierBasedSearch = T #T
    manual = F #F
    angle = randint(0, 628)/100.0
    rbt = routine()
    needToFalk = False
    #while(True):
        #print("New Twist")
    #rbt.makeTurnAround()
    #abc = input("Start?")
    if (not manual):
        #waypoint_list = rbt.makeTurnAround()
        rbt.needPause = True
        #walkRoutine(waypoint_list)
        rbt.makeTurnAround()
        rbt.needPause = False
    #if (rbt.findFrontier()): print(rbt.frontiers)
    while(True):

        #rbt.findFrontier()
        pos = rbt.getRobotCoordinates()
        print("B :", rbt.barbieFound)
        print("R: ", pos)
        #print("F:", rbt.closF_x, rbt.closF_y)
        #print("TG: ", round(((pos[0] + rbt.closF_x)/2),3), round(((pos[1] + rbt.closF_y)/2),3))
        waypoint_list = []
        if (not rbt.barbieFound):
            if (frontierBasedSearch):
                anythingFound = False
                print("Finding fronts...")
                while(rbt.findFrontier()):
                    print("Front return True")
                    if (rbt.barbieFound): break
                    dist_front, ind_front = spatial.KDTree(rbt.pointsVisited).query([rbt.closF_x, rbt.closF_y])
                    print("D:", dist_front)
                    if (dist_front > 0.655):
                        if (rbt.getpath(rbt.closF_x, rbt.closF_y)):
                            print("FRONTIER")
                            anythingFound = True
                            waypoint_list = rbt.create_trajectory()
                            needToFalk = True
                            rbt.needPause = True
                            #x = ""
                            #x = input("Can we go please? y/n: ")
                            #if (x.lower() == "y"):
                            if (True):
                                walkRoutine(waypoint_list)
                                dist_we_travelled = math.sqrt((rbt.lastPointInPath[0] - rbt.closF_x)**2 + (rbt.lastPointInPath[1] - rbt.closF_y)**2)
                                print("Till the point: ", dist_we_travelled)
                                if(dist_we_travelled <= 0.6): rbt.pointsVisited = np.vstack((rbt.pointsVisited, [rbt.closF_x, rbt.closF_y]))
                                rbt.needPause = False
                                if (needToFalk and not rbt.barbieFound):
                                    needToFalk = False
                                    #waypoint_list = rbt.makeTurnAround()
                                    rbt.needPause = True
                                    #walkRoutine(waypoint_list)
                                    rbt.makeTurnAround()
                                    rbt.needPause = False
                if (anythingFound == False):
                    randomWalks = True #T
                    frontierBasedSearch = False
                    manual = True
                    print("NO MORE Frnt, now Random")

            if (randomWalks):
                new_points = [pos[0] + movement*math.cos(angle), pos[1] + movement*math.sin(angle)]
                print("RW ANGLE: ", angle, "  ", math.degrees(angle))
                if(rbt.getpath(new_points[0], new_points[1])):
                    #pass
                    print("RANDOMWALK")
                    waypoint_list = rbt.create_trajectory()
                    needToFalk = True
                    x = ""
                    x = input("Can we go please? y/n: ")
                    if (x.lower() == "y"):
                        rbt.needPause = True
                        walkRoutine(waypoint_list)
                        rbt.needPause = False
                else:
                    angle = randint(0, 628)/100.0
                    #walkRoutine(waypoint_list)
                    if (needToFalk and not rbt.barbieFound):
                        needToFalk = False
                        print("GOING TO BARBIE")
                        #waypoint_list = rbt.makeTurnAround()
                        rbt.needPause = True
                        rbt.makeTurnAround()
                        #walkRoutine(waypoint_list)
                        rbt.needPause = False

        if (manual == True):
            while(True):
                try:
                    x = input("Next x, y:")
                    if (x == "-1"): sys.exit()
                    if(rbt.getpath(x[0], x[1])):
                        waypoint_list = rbt.create_trajectory()
                        y = input("Should we go? y/n")
                        if (y.lower() == "y"):
                            walkRoutine(waypoint_list)
                except:
                    manual = False
                    break

        elif (rbt.barbieFound):
            print("B: ", rbt.toGotoBarbie)
            if(rbt.getpath(rbt.toGotoBarbie[0], rbt.toGotoBarbie[1])):
                waypoint_list = rbt.create_trajectory()
                walkRoutine(waypoint_list)
                pos = rbt.getRobotCoordinates()
                dist = math.sqrt((pos[0]- rbt.toGotoBarbie[0])**2 + (pos[1]-rbt.toGotoBarbie[1])**2)
                print(dist)
                if (dist <= 0.5): break
            elif rbt.getpath(pos[0] + (rbt.toGotoBarbie[0] - pos[0])/2, pos[1] + (rbt.toGotoBarbie[1] - pos[1])/2):
                waypoint_list = rbt.create_trajectory()
                walkRoutine(waypoint_list)
                pos = rbt.getRobotCoordinates()
                dist = math.sqrt((pos[0]- rbt.toGotoBarbie[0])**2 + (pos[1]-rbt.toGotoBarbie[1])**2)
                print(dist)
                if (dist <= 0.5): break
                #rbt.toGotoBarbie[0] = rbt.barbiePoint.x
                #rbt.toGotoBarbie[0] = rbt.barbiePoint.y
            #rbt.barbieFound = False



        #x = x.split(',')

        #
        #while(rbt.findFrontier):
        #    rbt.getpathF()

            #


if __name__ == '__main__':

    #rbt.getpath(0,0)
    print('before main')
    main()
    rospy.spin()
