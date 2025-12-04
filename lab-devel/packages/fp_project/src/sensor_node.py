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

class SensorObjectDetector:
    def __init__(self):
        rospy.Subscriber("/ee483mm08/front_center_tof_driver_node/range", Range, self.range_publisher)
        self.pub_cmd = rospy.Publisher("/ee483mm08/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.pub = rospy.Publisher('/ee483mm08/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size = 10)
        self.v = 0
        self.omega = 0
        self.turns = 0
        self.state = "turning"
        self.avoiding = False

    def range_publisher(self, msg):
        value = msg.range
        rospy.loginfo(value)

        if value < 0.3:
            rospy.loginfo("Object Detected within 0.3 meters!")
            self.movement_control("stop")

        else:
            self.movement_control("move")

    def avoid_object(self, state):

        cmd_to_publish = WheelsCmdStamped()
        while self.avoiding:
        try:
            if self.state == 'forward':
                cmd_to_publish.header.stamp = rospy.Time.now()
                cmd_to_publish.vel_right = self.speed
                cmd_to_publish.vel_left = self.speed
                self.pub.publish(cmd_to_publish)
                rospy.sleep(5)

                if self.turns >= 4:
                    self.state = 'stop'
                
                self.state = 'pause_F'
            
            elif self.state == 'turn_left':
                # First stop
                self.turns += 1

                # Now turn
                cmd_to_publish.header.stamp = rospy.Time.now()
                cmd_to_publish.vel_right = -0.2
                cmd_to_publish.vel_left = 0.3
                self.pub.publish(cmd_to_publish)
                rospy.sleep(0.7)

                self.state = 'pause_T'
                
                if self.turns >= 4:
                    self.state = 'stop'
            
            elif self.state == 'turn_right':
                # First stop
                self.turns += 1

                # Now turn
                cmd_to_publish.header.stamp = rospy.Time.now()
                cmd_to_publish.vel_right = 0.3
                cmd_to_publish.vel_left = -0.2 
                self.pub.publish(cmd_to_publish)
                rospy.sleep(0.7)

                self.state = 'pause_T'
                
                if self.turns >= 4:
                    self.state = 'stop'

            elif self.state == 'pause_F':
                cmd_to_publish.header.stamp = rospy.Time.now()
                cmd_to_publish.vel_right = 0
                cmd_to_publish.vel_left = 0
                self.pub.publish(cmd_to_publish)
                rospy.sleep(2)

                self.state = 'turning'

            elif self.state == 'pause_T':
                cmd_to_publish.header.stamp = rospy.Time.now()
                cmd_to_publish.vel_right = 0
                cmd_to_publish.vel_left = 0
                self.pub.publish(cmd_to_publish)
                rospy.sleep(2)

                self.state = 'forward'

            elif self.state == 'stop':
                # cmd_to_publish.header.stamp = rospy.Time.now()
                # cmd_to_publish.vel_right = 0
                # cmd_to_publish.vel_left = 0
                # self.pub.publish(cmd_to_publish)
                # rospy.sleep(0.5)
                self.avoiding = False
                return
            
            # elif self.state == 'final_stop':
            #     cmd_to_publish.header.stamp = rospy.Time.now()
            #     cmd_to_publish.vel_right = 0
            #     cmd_to_publish.vel_left = 0
            #     self.pub.publish(cmd_to_publish)
            #     self.state = 'finished'
            
        except:
            rospy.loginfo("Except running...")
            cmd_to_publish.header.stamp = rospy.Time.now()
            cmd_to_publish.vel_right = 0
            cmd_to_publish.vel_left = 0
            self.pub.publish(cmd_to_publish)




    def movement_control(self, command):

        # check if command is a message of type Twist2DStamped
        # if isinstance(command, Twist2DStamped):
        #     v = command.v
        #     omega = command.omega
        #     if v == 0 and omega == 0:
        #         command = "stop"
        #     else:
        #         command = "move" 


        # rosparam set <parameter_name> <value>

        if rospy.has_param("velocity"):
            #Get the values of "kp" and store it in self.kp
            v = rospy.get_param("velocity")

        car_cmd = Twist2DStamped()
        
        if command == "stop":
            car_cmd.v = 0
            car_cmd.omega = 0
            self.pub_cmd.publish(car_cmd)
            rospy.loginfo("Published stop command to car_cmd_switch_node")
            
            self.avoiding = True
            self.avoid_object(self)
        
        
        elif command == "move" and self.turns == 0:
            car_cmd.v = self.v
            car_cmd.omega = self.omega if self.omega != 0 else 0.0
            self.pub_cmd.publish(car_cmd)
            rospy.loginfo("Published move command to car_cmd_switch_node")


if __name__ == "__main__":
    
    rospy.init_node("SensorObjectDetector", anonymous=True)
    object_detector = SensorObjectDetector()
    rospy.spin()