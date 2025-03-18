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
import time


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

    def process_image(self, cv_img):
        """ 
        @brief Process the image to detect the path and compute velocity commands 
        @param cam_raw contains image data in ROS image format
        @return velocity Twist message containing data for movement adjusted for current and desired trajectory
        @return processed_image a NumPy array of the binarized image, for display and debugging purposes
        """
        try:
            img = cv2.medianBlur(cv_img,5)

            # Convert BGR to HSV
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            uh = 130
            us = 255
            uv = 255
            lh = 110
            ls = 50
            lv = 50
            lower_hsv = np.array([lh,ls,lv])
            upper_hsv = np.array([uh,us,uv])

            # Threshold the HSV image to get only blue colors
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
            window_name = "HSV Calibrator"
            cv2.namedWindow(window_name)

            def nothing(x):
                print("Trackbar value: " + str(x))
                pass

            # create trackbars for Upper HSV
            cv2.createTrackbar('UpperH',window_name,0,255,nothing)
            cv2.setTrackbarPos('UpperH',window_name, uh)

            cv2.createTrackbar('UpperS',window_name,0,255,nothing)
            cv2.setTrackbarPos('UpperS',window_name, us)

            cv2.createTrackbar('UpperV',window_name,0,255,nothing)
            cv2.setTrackbarPos('UpperV',window_name, uv)

            # create trackbars for Lower HSV
            cv2.createTrackbar('LowerH',window_name,0,255,nothing)
            cv2.setTrackbarPos('LowerH',window_name, lh)

            cv2.createTrackbar('LowerS',window_name,0,255,nothing)
            cv2.setTrackbarPos('LowerS',window_name, ls)

            cv2.createTrackbar('LowerV',window_name,0,255,nothing)
            cv2.setTrackbarPos('LowerV',window_name, lv)

            font = cv2.FONT_HERSHEY_SIMPLEX

            print("Loaded images")

            while(1):
                # Threshold the HSV image to get only blue colors
                mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
                cv2.putText(mask,'Lower HSV: [' + str(lh) +',' + str(ls) + ',' + str(lv) + ']', (10,30), font, 0.5, (200,255,155), 1, cv2.LINE_AA)
                cv2.putText(mask,'Upper HSV: [' + str(uh) +',' + str(us) + ',' + str(uv) + ']', (10,60), font, 0.5, (200,255,155), 1, cv2.LINE_AA)

                cv2.imshow(window_name,mask)

                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    break
                # get current positions of Upper HSV trackbars
                uh = cv2.getTrackbarPos('UpperH',window_name)
                us = cv2.getTrackbarPos('UpperS',window_name)
                uv = cv2.getTrackbarPos('UpperV',window_name)
                upper_blue = np.array([uh,us,uv])
                # get current positions of Lower HSCV trackbars
                lh = cv2.getTrackbarPos('LowerH',window_name)
                ls = cv2.getTrackbarPos('LowerS',window_name)
                lv = cv2.getTrackbarPos('LowerV',window_name)
                upper_hsv = np.array([uh,us,uv])
                lower_hsv = np.array([lh,ls,lv])

                time.sleep(.1)

            cv2.destroyAllWindows()

            

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        node = PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass