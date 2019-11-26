#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import rospy
from geometry_msgs.msg import Pose2D, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from exploration.srv import PlanPath,  PlanPathRequest, PlanPathResponse
import numpy as np


if __name__ == "__main__":
    rospy.init_node("path_tester")
    publisher = rospy.Publisher("path", MarkerArray, queue_size=10)

    xPos = rospy.get_param("~x", -0.2)
    yPos = rospy.get_param("~y", 0.7)

    rospy.wait_for_service("plan_path")
    caller = rospy.ServiceProxy("plan_path", PlanPath)

    request = PlanPathRequest(Pose2D(xPos, yPos, 0.0))
    response = caller.call(request)
    rospy.loginfo(response)

    rospy.sleep(0.5)
    header = Header(stamp=rospy.Time.now(), frame_id="map")
    msg = MarkerArray([Marker(header=header, pose=Pose(position=Point(p.x, p.y, 0)), id=np.random.randint(0, 1000), type=1, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 0.5, 1, 1)) for p in response.path])
    publisher.publish(msg)
