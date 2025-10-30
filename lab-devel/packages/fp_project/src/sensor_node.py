#!/usr/bin/env python3

from sensor_msgs.msg import Range
import rospy

class SensorObjectDetector:
    def __init__(self):
        rospy.Subscriber("/ee483mm08/front_center_tof_driver_node/range", Range, self.range_publisher)

    def range_publisher(self, msg):
        value = msg.range
        print(value)

if __name__ == "__main__":
    
    rospy.init_node("SensorObjectDetector", anonymous=True)
    object_detector = SensorObjectDetector()
    rospy.spin()