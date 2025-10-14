#!/usr/bin/env python3
import rospy
import os
from duckietown_msgs.msg import WheelsCmdStamped # Import the message for the wheel comm

class Driver():#CHANGE CLASSNAME to the name of your class
    def __init__(self):
        self.veh_name = os.environ['VEHICLE_NAME']
        self.pub = rospy.Publisher('/ee483mm08/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size = 10)
        self.speed =  0.2 #was 0.5
        self.state = 'forward'
        self.turns = 0
        #self.subscriber = 'duckietown_msgs/WheelsCmdStamped'
            # USING PARAMETER TO GET THE NAME OF THE VEHICLE
            # THIS WILL BE USEFUL TO SPECIFY THE NAME OF THE TOPIC
            # INITIALIZE YOUR VARIABLES HERE (SUBSCRIBERS OR PUBLISHERS)
    
# calibrate and fix
    def drive(self): # CHANGE TO THE NAME OF YOUR FUNCTION
        print("running function")
        print("waiting to start....\n")
        cmd_to_publish = WheelsCmdStamped()
        print(self.state, self.speed)

        try:
            if self.state == 'forward':
                cmd_to_publish.header.stamp = rospy.Time.now()
                cmd_to_publish.vel_right = self.speed
                cmd_to_publish.vel_left = self.speed
                self.pub.publish(cmd_to_publish)
                rospy.sleep(5)

                if self.turns >=4:
                    self.state = 'stop'
                
                self.state = 'pause_F'
            
            elif self.state == 'turning':
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

            elif self.state == 'pause_F':
                cmd_to_publish.header.stamp = rospy.Time.now()
                cmd_to_publish.vel_right = 0
                cmd_to_publish.vel_left = 0
                self.pub.publish(cmd_to_publish)
                rospy.sleep(5)

                self.state = 'turning'

            elif self.state == 'pause_T':
                cmd_to_publish.header.stamp = rospy.Time.now()
                cmd_to_publish.vel_right = 0
                cmd_to_publish.vel_left = 0
                self.pub.publish(cmd_to_publish)
                rospy.sleep(5)

                self.state = 'forward'

            elif self.state == 'stop':
                cmd_to_publish.header.stamp = rospy.Time.now()
                cmd_to_publish.vel_right = 0
                cmd_to_publish.vel_left = 0
                self.pub.publish(cmd_to_publish)
                rospy.sleep(0.5)
            
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

#WRITE THE CODE TO MAKE THE MM GO AROUND THE BLOCK
if __name__ == "__main__": ## The main function which will be called when your python sc
# Initialize the node
    try:
        rospy.init_node('driving')
        drive = Driver() # Create obj of the Driver class
        rospy.sleep(8) # Delay to wait enough time for the code to run
        # Keep the line above - you might be able to reduce the delay a bit,
        while not rospy.is_shutdown() and drive.state != 'finished': # Run ros forever - you can change
            # this as well instead of running forever
            drive.drive() # calling your node function
    except rospy.ROSInterruptException:
        pass



# '''
#     def drive(self): # CHANGE TO THE NAME OF YOUR FUNCTION
#         print("running function")
#         cmd_to_publish = WheelsCmdStamped()
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right = 0
#         cmd_to_publish.vel_left = 0
#         self.pub.publish(cmd_to_publish)
# '''

#     '''
#     def drive(self):
#         print("running function")
#         print("waiting to start...\n")
#         cmd_to_publish = WheelsCmdStamped()
#         print(self.state, self.speed)

#         #First drive straight
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         self.state = 'forward'
#         cmd_to_publish.vel_right = self.speed
#         cmd_to_publish.vel_left = self.speed
#         self.pub.publish(cmd_to_publish)
#         rospy.sleep(2)

#         #First Stop
#         self.state = 'stop'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right=0
#         cmd_to_publish.vel_left=0
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)

#         #First turn
#         self.state = 'turn'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right = 0
#         cmd_to_publish.vel_left=0.2
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)

#         #second Stop
#         self.state = 'stop'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right=0
#         cmd_to_publish.vel_left=0
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)

#         #Second forward
#         self.state = 'forward'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right = 0.4
#         cmd_to_publish.vel_left = 0.4
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)

#         #third stop
#         self.state = 'stop'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right = 0
#         cmd_to_publish.vel_left = 0
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)

#         #Second turn
#         self.state = 'turn'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right = 0
#         cmd_to_publish.vel_left = 0.3
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)

#         #fourth Stop
#         self.state = 'stop'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right=0
#         cmd_to_publish.vel_left=0
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)

#         #Third forward
#         self.state = 'forward'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right = 0.4
#         cmd_to_publish.vel_left = 0.4
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)

#         #fifth stop
#         self.state = 'stop'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right = 0
#         cmd_to_publish.vel_left = 0
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)

#         #Third turn
#         self.state = 'turn'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right = -0.15
#         cmd_to_publish.vel_left = 0.3
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)

#         #sixth Stop
#         self.state = 'stop'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right=0
#         cmd_to_publish.vel_left=0
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)

#         #Fourth forward
#         self.state = 'forward'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right = 0.4
#         cmd_to_publish.vel_left = 0.4
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)


#         # Stop!
#         self.state = 'finished'
#         cmd_to_publish.header.stamp = rospy.Time.now()
#         cmd_to_publish.vel_right = 0
#         cmd_to_publish.vel_left = 0
#         self.pub.publish(cmd_to_publish)
#         print(self.state, self.speed)
#         rospy.sleep(2)
#     '''