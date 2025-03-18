#!/usr/bin/env python3
"""
@file image_to_velocity.py
@brief steers robot along path (line following)
"""

import rospy
from geometry_msgs.msg import Twist
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

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/B1/pi_camera/image_raw', Image, self.image_callback)

        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=10)

        # Publisher for score tracking
        self.score_pub = rospy.Publisher('/score_tracker', String, queue_size=1)

        # # Publisher for debugging image
        # self.debug_img_pub = rospy.Publisher('/debug/image_processed', Image, queue_size=1)

        self.bridge = CvBridge()

        self.twist = Twist()

    def image_callback(self, msg):
        """
        @brief A callback function to handle new incoming images. Converts ROS Image formats to OpenCV, calls the processing function to handle path finding and velocity adjustments. Also publishes a processed image for debugging purposes (shows a window of the camera's pov)
        @param msg contains image data
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the image and get velocity commands
            self.process_image(cv_image)

            # Publish velocity command
            # self.vel_pub.publish(velocity)

            # Publish processed image for debugging
            # debug_msg = self.bridge.cv2_to_imgmsg(processed_image, "mono8")
            # self.debug_img_pub.publish(debug_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", str(e))

    def process_image(self, cv_image):
        """ 
        @brief Process the image to detect the path and compute velocity commands 
        @param cam_raw contains image data in ROS image format
        @return velocity Twist message containing data for movement adjusted for current and desired trajectory
        @return processed_image a NumPy array of the binarized image, for display and debugging purposes
        """
        try:
            # Convert the ROS Image message to an OpenCV image
            # cv_image = self.bridge.imgmsg_to_cv2(cam_raw, desired_encoding="bgr8")

            height = cv_image.shape[0]
            width = cv_image.shape[1]
            blurred = cv2.blur(cv_image, (12, 12))
            # Convert to grayscale
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            road_lr = np.array([0, 0, 65])
            road_ur = np.array([0, 0, 90])
            mask = cv2.inRange(hsv, road_lr, road_ur)
            # Show the processed image

            row_of_interest = height - 75  # Line 75 pixels from the bottom
            line_data = mask[row_of_interest, 0:width]  # Extract the specific row
            right_found = False
            left_found = False

            leftmost = 0
            rightmost = width - 1
            print(line_data)

            for x in range(len(line_data)):
                # Get the leftmost and rightmost points
                if not left_found and line_data[x] == 255:
                    leftmost = x
                    left_found = True
                if not right_found and line_data[width - 2 - x] == 255:
                    rightmost = width - 1 - x
                    right_found = True
            if right_found or left_found:
                # Calculate the midpoint
                midpoint = (leftmost + rightmost) // 2

                # Find how far off we are from being centered
                error = midpoint - width/2

                rate = rospy.Rate(2)
                self.twist.linear.x = 0.5
                self.twist.angular.z = -error * 0.009

                # Publish movement command
                self.vel_pub.publish(self.twist)

                cv2.circle(cv_image, (int(midpoint), row_of_interest), 20, (255,0,0), -1)
            else:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.vel_pub.publish(self.twist)
                cv2.circle(cv_image, (int(width/2), row_of_interest), 20, (255,0,0), -1)
            
            cv2.imshow("White Line", cv_image)
            cv2.waitKey(1)

            cv2.imshow("Mask", mask)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        node = PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
