#!/usr/bin/env python3

import rospy
import os
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import WheelsCmdStamped

class DTJoystickDemoNode():
    def __init__(self):
        
        veh_name = os.environ['VEHICLE_NAME']
        self.sub_joy = rospy.Subscriber(
            f"/{veh_name}/joy", 
            Joy,
            self.process_joy
        )

        self.pub_wheel_cmds = rospy.Publisher(
            f"/{veh_name}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=10
        )

    def process_joy(self,msg):
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header = msg.header
        cmd_to_publish.vel_right = 0.0
        cmd_to_publish.vel_left = 0.0
        
        v = 0.5 # this will be the velocity we publish

        if msg.axes[1] == 1.0:
            # go forward
            cmd_to_publish.vel_right = v
            cmd_to_publish.vel_left = v

        if msg.axes[1] == -1.0:
            # go backwards
            cmd_to_publish.vel_right = -v
            cmd_to_publish.vel_left = -v

        if msg.axes[3] == -1.0:
            # turn right
            cmd_to_publish.vel_right = -v
            cmd_to_publish.vel_left = v

        if msg.axes[3] == 1.0:
            # turn left
            cmd_to_publish.vel_right = v
            cmd_to_publish.vel_left = -v

        self.pub_wheel_cmds.publish(cmd_to_publish)

if __name__ == "__main__":
    # Initialize the node
    node = DTJoystickDemoNode()
    rospy.init_node('dt_joystick_demo_node')
    # Keep it spinning
    rospy.spin()