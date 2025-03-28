#!/usr/bin/env python3
"""
@file   collect_data.py
@brief  saves images from camera to folder and names them
        based on lable
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os

class DataCollector:
    """
    @class DataCollector
    @brief Saves images from camera to folder and names them based on label
    """
    def __init__(self):
        """
        @brief Constructor for DataCollector
        """
        rospy.init_node('data_collector', anonymous=True)

        self.image_count = 0
        self.bridge = CvBridge()
        self.vel_linear = 0
        self.vel_angular = 0
        self.lable_ = ""

        self.type = input("Which terrain type are you collecting data for? (road, grass, off-road, tunnel, or ramp): ")
        self.type_terrain_folder = f'/home/fizzer/ENPH353_comp/ros_ws/src/controller_pkg/src/data/{self.type}'
        os.makedirs(self.type_terrain_folder, exist_ok=True)
        self.test_iteration = input("Which test iteration is this? (1, 2, 3, etc.): ")
        self.asset_output_folder = f'/home/fizzer/ENPH353_comp/ros_ws/src/controller_pkg/src/data/{self.type}/{self.test_iteration}'
        while(os.path.exists(self.asset_output_folder)):
            print(f"Iteration {self.test_iteration} already exists. Please enter a different test iteration.")
            self.test_iteration = input("Which test iteration is this? (1, 2, 3, etc.): ")
            self.asset_output_folder = f'/home/fizzer/ENPH353_comp/ros_ws/src/controller_pkg/src/data/{self.type}/{self.test_iteration}'
        print(f"Saving images to {self.asset_output_folder}")
        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/B1/pi_camera/image_raw', Image, self.image_callback)

        # Publisher for velocity commands
        self.vel_sub = rospy.Subscriber('/B1/cmd_vel', Twist, self.vel_callback)
        os.makedirs(self.asset_output_folder, exist_ok=True)
        
    def vel_callback(self, msg):
        """
        @brief  A callback function to handle new incoming velocity commands
        @param  msg contains velocity data
        """
        self.vel_linear = msg.linear.x
        self.vel_angular = msg.angular.z
    def image_callback(self, msg):
        """
        @brief A callback function to handle new incoming images. Converts ROS Image formats to OpenCV, calls the processing function to handle path finding and velocity adjustments. Also publishes a processed image for debugging purposes (shows a window of the camera's pov)
        @param msg contains image data
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the image and get velocity commands
            self.get_frame_and_lable(cv_image)

        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", str(e))

    def get_frame_and_lable(self, cv_image):
        """ 
        @brief Process the image to detect the path and compute velocity commands 
        @param cam_raw contains image data in ROS image format
        @return velocity Twist message containing data for movement adjusted for current and desired trajectory
        @return processed_image a NumPy array of the binarized image, for display and debugging purposes
        """
        try:
            # filename: [type of terrain][image count]_[linear velocity]_[angular velocity].png
            image_filename = f'iteration{self.test_iteration}_{self.type}{self.image_count}_linear_{self.vel_linear}_angular_{self.vel_angular}.png'
            cv2.imwrite(f'{self.asset_output_folder}/{image_filename}', cv_image)
            self.image_count += 1
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        node = DataCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
