#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import rospy
from geometry_msgs.msg import Pose2D, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from exploration.srv import AnyFrontiersLeft, AnyFrontiersLeftRequest, AnyFrontiersLeftResponse, GenerateFrontier, GenerateFrontierResponse
from exploration.srv import PlanPath,  PlanPathRequest, PlanPathResponse
import numpy as np

if __name__ == "__main__":
    rospy.init_node("frontier_planner")

    cmd = rospy.get_param("~cmd", "random")
    frontierMarker = rospy.Publisher("frontier", Marker, queue_size=10)
    pathMarkers = rospy.Publisher("path", MarkerArray, queue_size=10)
    rospy.wait_for_service("plan_path")
    pathPlanner = rospy.ServiceProxy("plan_path", PlanPath)


    if cmd.lower() == "random":
        rospy.wait_for_service("get_random_frontier")
        caller = rospy.ServiceProxy("get_random_frontier", GenerateFrontier)
        response = caller.call()
    elif cmd.lower() == "near":
        rospy.wait_for_service("get_closest_frontier")
        caller = rospy.ServiceProxy("get_closest_frontier", GenerateFrontier)
        response = caller.call()

    rospy.sleep(0.5)
    header = Header(stamp=rospy.Time.now(), frame_id="map")
    msg = Marker(header=header, pose=Pose(position=Point(response.goal_pose.x, response.goal_pose.y, 0)), id=np.random.randint(0, 1e9), type=Marker.CUBE, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 1, 0, 1), lifetime=rospy.Duration(0))
    frontierMarker.publish(msg)
    rospy.loginfo(response)

    request = PlanPathRequest(Pose2D(response.goal_pose.x, response.goal_pose.y, 0.0))
    response = pathPlanner.call(request)
    rospy.loginfo(response)

    rospy.sleep(0.5)
    msg = MarkerArray([Marker(header=header, pose=Pose(position=Point(p.x, p.y, 0)), id=np.random.randint(0, 1000), type=1, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 0.5, 1, 1)) for p in response.path])
    pathMarkers.publish(msg)
