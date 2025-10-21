#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge 

class ImageFlipper:
    def __init__(self):
        #Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("/ee483mm08/camera_node/image/compressed", CompressedImage, self.create_filter)
        self.pub_white = rospy.Publisher("white", Image, queue_size=10)
        self.pub_yellow = rospy.Publisher("yellow", Image, queue_size=10)
        self.pub_edges_white = rospy.Publisher("edges_white", Image, queue_size=10)
        self.pub_edges_yellow = rospy.Publisher("edges_yellow", Image, queue_size=10)

    def flipper_cb(self,msg):
        #convert to a ROS image using the bridge
        
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg,"bgr8")

        #flip along the horizontal axis using an OpenCV function
        cv_flipped = cv2.flip(cv_img, 1)

        #convert new image to ROS to send
        ros_flipped = self.bridge.cv2_to_imgmsg(cv_flipped, "bgr8")

        #publish flipped image
        self.pub.publish(ros_flipped)

    def create_filter(self, msg):
        print("I got here")
        #convert to a ROS image using the bridge
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # create the yellow mask
        cv_mask_yellow = cv2.inRange(hsv_img, (0,103,140), (59,255,255))

        # create the white mask
        cv_mask_white = cv2.inRange(hsv_img, (70,0,145), (140,120,255))

        #convert new image to ROS to send

        edges = cv2.Canny(hsv_img, 100, 200)
        white_boundaries = cv2.bitwise_and(cv_mask_white, edges)
        yellow_boundaries = cv2.bitwise_and(cv_mask_yellow, edges)

        ros_masked_yellow = self.bridge.cv2_to_imgmsg(cv_mask_yellow, "mono8")
        ros_masked_white = self.bridge.cv2_to_imgmsg(cv_mask_white, "mono8")
        ros_white_boundaries = self.bridge.cv2_to_imgmsg(white_boundaries, "mono8")
        ros_yellow_boundaries = self.bridge.cv2_to_imgmsg(yellow_boundaries, "mono8")

        #publish flipped image
        self.pub_white.publish(ros_masked_white)
        self.pub_yellow.publish(ros_masked_yellow)
        self.pub_edges_white.publish(ros_white_boundaries)
        self.pub_edges_yellow.publish(ros_yellow_boundaries)


if __name__=="__main__":
    #initialize our node and create a publisher as normal
    rospy.init_node("image_flipper", anonymous=True)
    img_flip = ImageFlipper()
    rospy.spin()


#Changes we need to make
# subscribe to correct camera topic name
# import CompressedImage message type
# import os
# Create subscriber for CompressedImage
# Change conversion method used to CompressedImage instead of Image
# Sync the image in the MM