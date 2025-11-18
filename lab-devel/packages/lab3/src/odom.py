#!/usr/bin/env python3



import sys
import rospy
import cv2
import matplotlib
import numpy as np
import os
from message_filters import ApproximateTimeSynchronizer, Subscriber
from duckietown_msgs.msg import Segment, SegmentList, Vector2D
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_srvs.srv import SetBool, SetBoolResponse
from duckietown_msgs.msg import LanePose, Twist2DStamped
from odom_aux.msg import DistWheel, Pose2D
import math


#
# We didn't end up with much time to work more on this, so this is as far as we got.
#


class Odometry:
    def __init__(self):
        controller_sub = rospy.Subscriber("ee483mm08/left_wheel_encoder_node/tick", DistWheel, self.odom)
        controller_sub = rospy.Subscriber("ee483mm08/right_wheel_encoder_node/tick", DistWheel, self.odom)
        self.pub_pose = rospy.Publisher("ee483mm08/distance_traveled", Pose2D, queue_size=10)
        self.N = 0
        self.N_total = 0
        self.L = 0.1/2
        self.x = 0
        self.y = 0
        self.theta = 0 


    def odom(self, msg):
        left_wheel = msg.dist_wheel_left
        right_wheel = msg.dist_wheel_right

        delta_s = (left_wheel + right_wheel)/2
        delta_theta = (right_wheel - left_wheel)/(2*self.L)

        delta_x = delta_s*math.cos(self.theta + (delta_theta/2))
        delta_y = delta_s*math.sin(self.theta + (delta_theta/2))

        self.x = self.x + delta_x
        self.y = self.y + delta_y
        self.theta = self.theta + delta_theta

        msg_pose = Pose2D()
        msg_pose.x = self.x
        msg_pose.y = self.y
        msg_pose.theta = self.theta
        self.pub_pose.publish(msg_pose)

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    # time.sleep(5)
    rospy.init_node("odom", anonymous=True)
    controller = Odometry()
    rospy.spin()