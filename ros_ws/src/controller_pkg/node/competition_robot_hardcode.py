#!/usr/bin/env python3
"""
@file image_to_velocity.py
@brief steers robot along path (line following) using PID
"""

# Imports for Moving Robot and Vision
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os

rospy.init_node('competition_robot_hardcode', anonymous=True) # change in your launch file!
pub_vel = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=10)
pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
print("initialized++++++++++++++++++++++++++++")
move = Twist()

rospy.sleep(1)


pub_score.publish('fabs,nopassword,0,NA')

def spawn_position(position):

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


########### GRASS ################

spawn_position([1.55, -0.812, 0.04, 0, 0, -0.487])

move.linear.x = 0.8
move.angular.z = 0
pub_vel.publish(move)

rospy.sleep(2)

move.linear.x = 0
move.angular.z = 2
pub_vel.publish(move)

rospy.sleep(2)

pub_score.publish('fabs,nopassword,0,NA')