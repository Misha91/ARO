#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import rospy
from geometry_msgs.msg import Pose2D, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from exploration.srv import AnyFrontiersLeft, AnyFrontiersLeftRequest, AnyFrontiersLeftResponse, GenerateFrontier, GenerateFrontierResponse
import numpy as np

if __name__ == "__main__":
    rospy.init_node("front_tester")

    cmd = rospy.get_param("~cmd", "near")
    publisher = rospy.Publisher("frontier", Marker, queue_size=10)

    if cmd.lower() == "any":
        rospy.wait_for_service("any_frontiers_left")
        caller = rospy.ServiceProxy("any_frontiers_left", AnyFrontiersLeft)
        rospy.loginfo(caller.call())
    else:
        if cmd.lower() == "random":
            rospy.wait_for_service("get_random_frontier")
            caller = rospy.ServiceProxy("get_random_frontier", GenerateFrontier)
            response = caller.call()
        elif cmd.lower() == "near":
            rospy.wait_for_service("get_closest_frontier")
            caller = rospy.ServiceProxy("get_closest_frontier", GenerateFrontier)
            response = caller.call()

        rospy.sleep(0.5)
        header = Header(frame_id="map")
        msg = Marker(header=header, pose=Pose(position=Point(response.goal_pose.x, response.goal_pose.y, 0)), id=np.random.randint(0, 1e9), type=Marker.CUBE, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 1, 0, 1), lifetime=rospy.Duration(0))
        publisher.publish(msg)
        rospy.loginfo(response)
