#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge 
import numpy as np

class CameraObjectDetector:
    def __init__(self):
        #Instatiate the converter class once by using a class member
        self.pub_white = rospy.Subscriber("white", Image, queue_size=10)
        self.pub_yellow = rospy.Publisher("yellow", Image, queue_size=10)
        self.pub_edges_white = rospy.Publisher("edges_white", Image, queue_size=10)
        self.pub_edges_yellow = rospy.Publisher("edges_yellow", Image, queue_size=10)
        self.pub_lines = rospy.Publisher("lines", Image, queue_size=10)

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
        cv_img = cv_img[240:480, 0:640]
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))

        # create the yellow mask
        cv_mask_pre_yellow = cv2.inRange(hsv_img, (15,40,40), (45,255,255))
        cv_mask_yellow = cv2.dilate(cv_mask_pre_yellow, kernel)

        # create the white mask
        cv_mask_pre_white = cv2.inRange(hsv_img, (70,0,145), (140,120,255))
        cv_mask_white = cv2.dilate(cv_mask_pre_white, kernel)


        #convert new image to ROS to send

        edges = cv2.Canny(hsv_img, 100, 200)
        white_boundaries = cv2.bitwise_and(cv_mask_white, edges)
        yellow_boundaries = cv2.bitwise_and(cv_mask_yellow, edges)

        ros_masked_yellow = self.bridge.cv2_to_imgmsg(cv_mask_yellow, "mono8")
        ros_masked_white = self.bridge.cv2_to_imgmsg(cv_mask_white, "mono8")
        ros_white_boundaries = self.bridge.cv2_to_imgmsg(white_boundaries, "mono8")
        ros_yellow_boundaries = self.bridge.cv2_to_imgmsg(yellow_boundaries, "mono8")
        white_lines = cv2.HoughLinesP(white_boundaries,1,np.pi/180,0,minLineLength=30,maxLineGap=10)
        yellow_lines = cv2.HoughLinesP(yellow_boundaries,1,np.pi/180,0,minLineLength=30,maxLineGap=10)
        
        #self.lines = [white_lines,yellow_lines]
        
        image_lines_w = self.output_lines(cv_img, white_lines, (0,255,0))
        image_lines = self.output_lines(image_lines_w, yellow_lines, (255,0,0))
        ros_image_lines = self.bridge.cv2_to_imgmsg(image_lines, "bgr8")

        #publish flipped image
        self.pub_white.publish(ros_masked_white)
        self.pub_yellow.publish(ros_masked_yellow)
        self.pub_edges_white.publish(ros_white_boundaries)
        self.pub_edges_yellow.publish(ros_yellow_boundaries)
        self.pub_lines.publish(ros_image_lines)

    
    def output_lines(self, cv_image, lines, color=(0,255,0)):
        output = np.copy(cv_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]),(l[2],l[3]), color, 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, color)
                cv2.circle(output, (l[2],l[3]), 2, color)
        return output


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