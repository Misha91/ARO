#!/usr/bin/env python
# -*- encoding: utf-8 -*-
# An example script to send a trajectory to a robot, start and stop the motion.
import time
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32
from robot_coordination.msg import Waypoint
from robot_coordination.srv import AddPath
from robot_coordination.srv import StartMovement
from robot_coordination.srv import StopMovement
import rospy

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

def create_trajectory():
    """ Create a trajectory in the odometry frame. """
    # x,y,time
    gtr = [
        [0.0, 0.0, 0],
        [0.15, 0.15, 4],
        [0.25, 0.15, 8],
        [0.25, 0.25, 12],
        [0.25, 0.35, 16],
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

def main():
    rospy.init_node('fetch_barbie')

    pf = PathFollowing()
    rate = rospy.Rate(10)
    waypoint_list = create_trajectory()
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


if __name__ == '__main__':
    main()
