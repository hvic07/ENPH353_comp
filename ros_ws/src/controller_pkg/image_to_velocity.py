#!/usr/bin/env python3
"""
@file image_to_velocity.py
@brief steers robot along path (line following)
"""

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
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

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/camera1/image_raw', Image, self.image_callback)

        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Publisher for debugging image
        self.debug_img_pub = rospy.Publisher('/debug/image_processed', Image, queue_size=1)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        """
        @brief A callback function to handle new incoming images. Converts ROS Image formats to OpenCV, calls the processing function to handle path finding and velocity adjustments. Also publishes a processed image for debugging purposes (shows a window of the camera's pov)
        @param msg contains image data
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the image and get velocity commands
            velocity, processed_image = self.process_image(cv_image)

            # Publish velocity command
            self.vel_pub.publish(velocity)

            # Publish processed image for debugging
            debug_msg = self.bridge.cv2_to_imgmsg(processed_image, "mono8")
            self.debug_img_pub.publish(debug_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", str(e))

    def process_image(self, cv_image):
        """ 
        @brief Process the image to detect the path and compute velocity commands 
        @param cv_image contains image data in OpenCV format
        @return velocity Twist message containing data for movement adjusted for current and desired trajectory
        @return processed_image a NumPy array of the binarized image, for display and debugging purposes
        """
        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Binarize: Keep only dark regions (assumed to be the path)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

        # Find contours of the path
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        velocity = Twist()

        if contours:
            # Find the largest contour (assuming it's the path)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get the center of the contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])  # X position of path center
                cy = int(M["m01"] / M["m00"])  # Y position (not used here)

                # Get image center
                height, width = binary.shape
                image_center = width // 2  # Middle of the image

                # Compute error: how far the path is from the center
                error = image_center - cx

                # Proportional control for steering
                Kp = 0.005  # Tune this value to adjust turning sensitivity
                velocity.linear.x = 0.2  # Constant forward speed
                velocity.angular.z = Kp * error  # Steer towards the path center

                # Draw center lines for debugging
                cv2.line(binary, (cx, 0), (cx, height), (128, 128, 128), 2)  # Path center
                cv2.line(binary, (image_center, 0), (image_center, height), (255, 255, 255), 2)  # Image center

        return velocity, binary

if __name__ == '__main__':
    try:
        node = PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
