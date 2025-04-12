#!/usr/bin/env python3
"""
@file image_to_velocity.py
@brief steers robot along path (line following) using PID
"""

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class PathFollower:
    """
    @class PathFollower
    @brief Uses camera inputs to steer robot in line following path
    """
    def __init__(self):
        """
        @brief Constructor for PathFollower
        """
        rospy.init_node('path_follower', anonymous=True)

        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=10)

        # Publisher for score tracking
        self.score_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        
        self.twist = Twist()
        rospy.sleep(2)
        self.move()

    def move(self):
        self.score_pub.publish('fabs,nopassword,0,NA')
        ############ ROAD #######################
        rospy.sleep(5)

        # initial full send
        self.twist.linear.x = 2.0
        self.twist.angular.z = 0.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(3.9)

        # pause for clue 2
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(3.0)

        # continue to bend
        self.twist.linear.x = 2.0
        self.twist.angular.z = 0.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(2.4)

        # turn around bend
        self.twist.linear.x = 0.0
        self.twist.angular.z = -2.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.85)

        self.twist.linear.x = 2.0
        self.twist.angular.z = 0.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.0)  

        self.twist.linear.x = 0.0
        self.twist.angular.z = -2.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.0)      

        # stop road sequence
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(4.0)  

        ############ SPAWN TO GRASS ROAD ########
        self.spawn_position([0.477889, -0.169225, 0.04, 0, 0, -0.7, -0.8732])
        
        # quick turn adjust
        self.twist.linear.x = 0.0
        self.twist.angular.z = 2.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(0.15)

        # quick rear run!
        self.twist.linear.x = -2.5
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.1)   

        # pause to read clue
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(2.0)     

        # send dis bitch!
        self.twist.linear.x = 4.0
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(2.2)

        # 180!
        self.twist.linear.x = 0
        self.twist.angular.z = -4.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.4)

        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(2.5)

        self.twist.linear.x = 0
        self.twist.angular.z = -4.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(0.5)

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(0.8)

        self.twist.linear.x = 4.0
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(0.45)

        self.twist.linear.x = 0.0
        self.twist.angular.z = -4.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.0)

        # SEND ACROSS BRIDGE!!!!
        self.twist.linear.x = 5.0
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.7)

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(3.0)

        self.spawn_position([-4.0921, -2.3015, 0.04, 0, 0, 0, -0.8732])

        rospy.sleep(0.5)

        ################ LONG STRETCH ########################

        # self.twist.linear.x = 0
        # self.twist.angular.z = -2
        # self.vel_pub.publish(self.twist)

        # rospy.sleep(1.67)

        # self.twist.linear.x = 1
        # self.twist.angular.z = 0
        # self.vel_pub.publish(self.twist)

        # rospy.sleep(10.15)

        # ############### BEGIN OFF ROAD #########################

        # self.twist.linear.x = 0
        # self.twist.angular.z = 2
        # self.vel_pub.publish(self.twist)

        # rospy.sleep(1.7)

        # self.twist.linear.x = 1
        # self.twist.angular.z = 0
        # self.vel_pub.publish(self.twist)

        # rospy.sleep(7.5)

        # self.twist.linear.x = 0
        # self.twist.angular.z = -2
        # self.vel_pub.publish(self.twist)

        # rospy.sleep(1)

        # self.twist.linear.x = 1
        # self.twist.angular.z = 0
        # self.vel_pub.publish(self.twist)

        # rospy.sleep(2)

        # ############### ALIGN TUNNEL ############################

        # self.twist.linear.x = 0
        # self.twist.angular.z = 2
        # self.vel_pub.publish(self.twist)

        # rospy.sleep(1.3)


        # self.twist.linear.x = 1
        # self.twist.angular.z = 0
        # self.vel_pub.publish(self.twist)

        # rospy.sleep(1.25)

        # self.twist.linear.x = 0
        # self.twist.angular.z = 2
        # self.vel_pub.publish(self.twist)

        # rospy.sleep(1.3)

        print('sleeping')
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)

        rospy.sleep(1.5)
        return
    
    def spawn_position(self, position):

        msg = ModelState()
        msg.model_name = 'B1'

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = position[3]
        msg.pose.orientation.y = position[4]
        msg.pose.orientation.z = position[5]
        msg.pose.orientation.w = position[6]

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( msg )

        except rospy.ServiceException:
            print ("Service call failed")


if __name__ == '__main__':
    try:
        node = PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

