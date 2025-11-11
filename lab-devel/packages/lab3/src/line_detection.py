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


class ImageProcessor: 
    def __init__(self):
        # Instatiate the converter class once by using a class member
        rospy.Service('line_detector_node/switch', SetBool, self.ld_switch)
        rospy.Service('lane_filter_node/switch',SetBool, self.lf_switch)
        self.bridge = CvBridge()
        veh_name = os.environ['VEHICLE_NAME']
        rospy.Subscriber(f"/{veh_name}/camera_node/image/compressed", CompressedImage, self.processor, queue_size=1, buff_size= 2**24)
        rospy.Subscriber(f"/{veh_name}//lane_filter_node/lane_pose", LanePose, self.lane_pose_callback, queue_size=1, buff_size= 2**24) #possibly self.lane_pose_callback, possible queue size and buff size
        #rospy.Subscriber("image", Image, self.processor, queue_size=1, buff_size= 2**24)
        self.pub_edges = rospy.Publisher("image_edges", Image, queue_size=10)
        self.pub_yellow_mask = rospy.Publisher("image_mask_yellow", Image, queue_size=10)
        self.pub_white_mask = rospy.Publisher("image_mask_white", Image, queue_size=10)
        self.pub_yellow_lines = rospy.Publisher("image_lines_yellow", Image, queue_size=10)
        self.pub_white_lines = rospy.Publisher("image_lines_white", Image, queue_size=10)
        self.pub_lines = rospy.Publisher("image_lines", Image, queue_size=10)
        self.pub_segments = rospy.Publisher("line_detector_node/segment_list", SegmentList, queue_size=10)
        self.pub_lane_pose = rospy.Publisher("/ee483mm08/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        
        #variables used in class
        self.hsv_image = None
        self.cv_img = None
        self.cv_crop = None
        self.white_mask = None
        self.yellow_mask = None

        # Default PID params
        self.cumulative_error = 0
        self.dt = 0
        self.last_error = 0
        self.ki = 0
        self.kp = 5
        self.kd = 0

    def lane_pose_callback(self, data):
        rospy.loginfo("Received data from lane pose node: %s", data.phi)

        error = (-1)*data.phi
        
        # Check if parameter "kp" exists first
        if rospy.has_param("kp"):
            #Get the values of "kp" and store it in self.kp
            self.kp = rospy.get_param("kp")

        if rospy.has_param("ki"):
            #Get the values of "ki" and store it in self.ki
            self.kp = rospy.get_param("ki")

        if rospy.has_param("kd"):
            #Get the values of "kd" and store it in self.kd
            self.kp = rospy.get_param("kd")

        controller_output = self.PID(error)
        
        car_cmd = Twist2DStamped()
        car_cmd.v = 0
        car_cmd.omega = controller_output

        self.pub_lane_pose.publish(car_cmd)
        
    def integral(self, value, time):
        self.cumulative_error = self.cumulative_error + (value * time)
        return self.cumulative_error

    def derivative(self, last_value, value, time):
        d = (value - last_value)/ time
        return d

    def PID(self, data):
        value = data
        time = rospy.get_rostime().nsecs
        time_diff = self.dt - time
        i = self.integral(value, time_diff)
        d = self.derivative(self.last_error, value, time_diff)
        
        pid = self.kp*value + self.ki*i + self.kd*d


        self.last_error = value
        self.dt = time
        return pid

    def processor(self, msg):
        self.cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        #self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.crop() #Crop Image
        self.hsv_image = cv2.cvtColor(self.cv_crop, cv2.COLOR_BGR2HSV)
        self.white_filter() #Create white filter
        self.yellow_filter() #create yellow filter
        self.segment() #Create segments from filters

    # DO NOT CHANGE THIS FUNCTION
    def crop(self):
        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(self.cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        self.cv_crop= new_image[offset:, :]

    def white_filter(self):
        #Finds all pixels within the given range. In this scenario its red
        # CHANGE TO YOUR VALUES

        mask = cv2.inRange(self.hsv_image, (70,0,145),(140,120,255))
        # mask2 = cv2.inRange(self.hsv_image, (13,0,154),(33,255,255))
        # mask = cv2.bitwise_or(mask,mask2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        # image_erode = cv2.erode(red_part, kernel)
        image_erode = cv2.erode(mask, kernel)
        image_dilate = cv2.dilate(image_erode, kernel2)
        self.white_mask = image_dilate
        #Converting image from cv format to ROS msg using 
        white_mask = self.bridge.cv2_to_imgmsg(self.white_mask, "mono8")
        #publishing cropped image
        self.pub_white_mask.publish(white_mask)


    def yellow_filter(self):
        # CHANGE VALUES IF NEEDED
        #Finds all pixels within the given range. In this scenario its red
        mask = cv2.inRange(self.hsv_image, (15,40,40), (45,255,255))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        # image_erode = cv2.erode(red_part, kernel)
        image_erode = cv2.erode(mask, kernel)
        image_dilate = cv2.dilate(image_erode, kernel2)
        self.yellow_mask = image_dilate
        #Converting image from cv format to ROS msg using 
        white_mask = self.bridge.cv2_to_imgmsg(self.yellow_mask, "mono8")
        #publishing cropped image

        self.pub_yellow_mask.publish(white_mask)

    def segment(self):
        #Convert image from BGR to HSV
        # CHANGE VALUES TO YOUR VALUES

        edges = cv2.Canny(self.cv_crop, 100, 200)

        #Combinining edges with mask white 
        edges_white = cv2.bitwise_and(edges,self.white_mask)
        ros_edges = self.bridge.cv2_to_imgmsg(edges_white, "mono8")
        self.pub_white_lines.publish(ros_edges)

        #Combinining edges with mask yellow 
        edges_yellow = cv2.bitwise_and(edges,self.yellow_mask)
        ros_edges = self.bridge.cv2_to_imgmsg(edges_yellow, "mono8")
        self.pub_yellow_lines.publish(ros_edges)

        
        #Finding Line Segments
        # CHANGE VALUES TO YOUR VALUES
        hough_white = cv2.HoughLinesP(edges_white, 1, np.pi / 180, 0,minLineLength=30,maxLineGap=10)
        
        # CHANGE VALUES IF NEEDED 
        hough_yellow = cv2.HoughLinesP(edges_yellow, 1, np.pi / 180, 0,minLineLength=30,maxLineGap=10)

        # DO NOT CHANGE
        #Publish and  Segments
        final_lines = self.output_lines(self.cv_crop,hough_white,hough_yellow)
        ros_lines = self.bridge.cv2_to_imgmsg(final_lines, "bgr8")
        self.pub_lines.publish(ros_lines)
        self.convert_lines(hough_white,hough_yellow)


    def convert_lines(self,lines_white,lines_yellow):
        if lines_white is not None:
            lines_reshaped =lines_white.reshape((-1,4))
        else:
            lines_reshaped =[]
        if lines_yellow is not None:
            lines_reshaped_yellow =lines_yellow.reshape((-1,4))
        else:
            lines_reshaped_yellow =[]
        offset = 40
        image_size = (160,120)
        arr_cutoff = np.array([0,offset, 0, offset])
        arr_ratio = np.array([([1. /image_size[0], 1. /image_size[1], 1. /image_size[0], 1. /image_size[1]])])
        if lines_white is not None:
            line_normalized_white = (lines_reshaped + arr_cutoff) * arr_ratio
        else:
            line_normalized_white= np.empty(shape= [0,0,])
            
        if lines_yellow is not None:
            line_normalized_yellow = (lines_reshaped_yellow + arr_cutoff) * arr_ratio
        else:
            line_normalized_yellow= np.empty(shape= [0,0,])
        
        Line_List = SegmentList()
        Line_Array = []
        # print(line_normalized_white)
        for i in range(line_normalized_white.shape[0]):
        # for i in range(1):

            line = Segment()
            line.color = line.WHITE
            Initial_Line = Vector2D(line_normalized_white[i,0], line_normalized_white[i,1] )
            End_Line = Vector2D(line_normalized_white[i,2], line_normalized_white[i,3] )
            line.pixels_normalized[0] = Initial_Line
            line.pixels_normalized[1] = End_Line
            Line_Array.append(line)
        for i in range(line_normalized_yellow.shape[0]):
        # for i in range(1):

            line = Segment()
            line.color = line.YELLOW
            Initial_Line = Vector2D(line_normalized_yellow[i,0], line_normalized_yellow[i,1] )
            End_Line = Vector2D(line_normalized_yellow[i,2], line_normalized_yellow[i,3] )
            line.pixels_normalized[0] = Initial_Line
            line.pixels_normalized[1] = End_Line
            Line_Array.append(line)
        Line_List.segments= Line_Array
        Line_List.header.stamp = rospy.Time.now()
        self.pub_segments.publish(Line_List)


    def output_lines(self, original_image, lines,lines_yellow):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        if lines_yellow is not None:
            for i in range(len(lines_yellow)):
                l = lines_yellow[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (0,0,255), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,255,255))
        return output
       
       
    
    def ld_switch(self,msg):
        return True,""
    
    def lf_switch(self,msg):
        return True,""



if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("ImageProcessor", anonymous=True)
    ImageProcessor()
    rospy.spin()