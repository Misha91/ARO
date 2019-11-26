#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox
from threading import Thread

class PoseSender():

    def __init__(self):
        xPos = rospy.get_param('~x', 0)
        yPos = rospy.get_param('~y', 0)
        mapFrame = rospy.get_param('map_frame', "map")
        robotFrame = rospy.get_param('robot_frame', "robot")

        self.poseSubscriber = rospy.Subscriber("robot_pose", geometry_msgs.msg.Pose2D, self.changeValue)

        self.br = tf2_ros.TransformBroadcaster()
        self.trans = geometry_msgs.msg.TransformStamped()

        self.trans.header.frame_id = mapFrame
        self.trans.child_frame_id = robotFrame
        self.trans.transform.translation.x = xPos
        self.trans.transform.translation.y = yPos
        self.trans.transform.rotation.w = 1.0

        self.rate = rospy.Rate(2)

    def changeValue(self, msg):
        self.trans.transform.translation.x = msg.x
        self.trans.transform.translation.y = msg.y


    def sendPoses(self):
        while not rospy.is_shutdown():
            self.trans.header.stamp = rospy.Time.now()
            self.br.sendTransform(self.trans)
            self.rate.sleep()


class PoseGetter():

    def __init__(self, xPos, yPos):
        self.xPos = xPos
        self.yPos = yPos
        self.posePublisher = rospy.Publisher("robot_pose", geometry_msgs.msg.Pose2D, queue_size=10)

        plt.ion()
        fig, axes = plt.subplots(nrows=2, figsize=(5, 2))
        text_box1 = TextBox(axes[0], 'X: ', initial=str(xPos))
        text_box1.on_submit(lambda text: self.sendChange(float(text), None))
        text_box2 = TextBox(axes[1], 'Y: ', initial=str(yPos))
        text_box2.on_submit(lambda text: self.sendChange(None, float(text)))
        plt.show(block=True)

    def sendChange(self, x, y):
        msg = geometry_msgs.msg.Pose2D()

        if x is not None:
            self.xPos = msg.x = x
        else:
            msg.x = self.xPos
        if y is not None:
            self.yPos = msg.y = y
        else:
            msg.y = self.yPos
        plt.draw()
        self.posePublisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node('pose_sender')
    ps = PoseSender()
    process = Thread(target=ps.sendPoses)
    process.setDaemon(True)
    process.start()
    pg = PoseGetter(ps.trans.transform.translation.x, ps.trans.transform.translation.y)
