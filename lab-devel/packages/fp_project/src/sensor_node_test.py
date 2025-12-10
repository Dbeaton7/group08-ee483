#!/usr/bin/env python3

from sensor_msgs.msg import Range
import rospy
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
from duckietown_msgs.msg import WheelsCmdStamped

class SensorObjectDetector:
    def __init__(self):
        rospy.Subscriber("/ee483mm08/front_center_tof_driver_node/range", Range, self.range_publisher)
        self.pub_cmd = rospy.Publisher("/ee483mm08/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.pub = rospy.Publisher('/ee483mm08/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size = 10)
        self.v = rospy.get_param("velocity") if rospy.has_param("velocity") else 0 
        self.omega = 0
        self.turns = 0
        self.avoiding = False

    def range_publisher(self, msg):
        value = msg.range
        rospy.loginfo(value)
        
        range_to_detect = rospy.get_param("range") if rospy.has_param("range") else 0.3

        if msg.range < range_to_detect:
            rospy.loginfo("Object Detected within 0.3 meters!")
            # self.movement_control("stop", value)
            
            car_cmd = Twist2DStamped()
            car_cmd.v = 0
            car_cmd.omega = rospy.get_param("omega") if rospy.has_param("omega") else 0.5

            self.pub_cmd.publish(car_cmd)
            rospy.loginfo(f"Avoiding object - Current range: {msg.range}")

        else:
            self.movement_control("move", value)

    def movement_control(self, command, range):

        # rosparam set <parameter_name> <value>

        #Get the values of "v" and store it in self.v
        self.v = rospy.get_param("velocity") if rospy.has_param("velocity") else 0

        car_cmd = Twist2DStamped()
        
        if command == "stop":
            car_cmd.v = 0
            car_cmd.omega = 0
            self.pub_cmd.publish(car_cmd)
            rospy.loginfo("Published stop command to car_cmd_switch_node" \
                            "and starting avoidance procedure")
            
            # self.state = "turn_left"
            self.avoiding = True
            self.avoid_object(range)
        
        
        elif command == "move" and self.turns == 0:
            car_cmd.v = self.v
            car_cmd.omega = self.omega if self.omega != 0 else 0.0
            self.pub_cmd.publish(car_cmd)
            rospy.loginfo("Published move command to car_cmd_switch_node")


    def avoid_object(self, range):

        range_to_detect = rospy.get_param("range") if rospy.has_param("range") else 0.3

        if range < range_to_detect:
            car_cmd = Twist2DStamped()
            car_cmd.v = 0
            car_cmd.omega = rospy.get_param("omega") if rospy.has_param("omega") else 0.5

            self.pub_cmd.publish(car_cmd)
            rospy.loginfo(f"Avoiding object - Current range: {range}")

if __name__ == "__main__":
    
    rospy.init_node("SensorObjectDetector", anonymous=True)
    object_detector = SensorObjectDetector()
    rospy.spin()