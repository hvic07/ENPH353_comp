#!/usr/bin/env python3
import numpy as np
import cv2

import rospy
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import os
import random
import shutil
import string
from operator import index
import matplotlib.pyplot as plt

# tensorflow 12.1
import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers
from tensorflow.keras.utils import plot_model
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.models import load_model # to introduce trained model

class ClueReader:
    '''
    @class PathFollower
    @brief Uses camera inputs to steer robot in line following path
    '''
    def __init__(self):
        """
        @brief Constructor for ClueReader
        """
        rospy.init_node('clue_reader', anonymous=True)

        # Load neural network
        self.clue_mod = load_model('/home/hannahcha/comp_enph/ros_ws/src/controller_pkg/src/clue_reader_models/clue_reader.h5')

        self.teamname = 'fabs'
        self.password = '9'

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/B1/pi_camera/image_raw', Image, self.image_callback)

        # Publisher to score_tracker 
        self.score_pub = rospy.Publisher('/score_tracker', String, queue_size=1)

        # # Publisher for debugging image
        # self.debug_img_pub = rospy.Publisher('/debug/image_processed', Image, queue_size=1)

        self.bridge = CvBridge() # for converting between cv2 and ros formats
    
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

            # Publish new clue command
            # self.vel_pub.publish()

            # Publish processed image for debugging
            # debug_msg = self.bridge.cv2_to_imgmsg(processed_image, "mono8")
            # self.debug_img_pub.publish(debug_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", str(e))
    
    def process_image(self, cv_image):
        """ 
        @brief Process the image to detect the clue board and output strings 
        @param cv_image contains image data in ROS image format
        @return String message containing clue data
        @return processed_image a NumPy array of the binarized image, for display and debugging purposes
        """
        # define lower and upper bounds for character mask
        lower_bound = np.array[(120, 120, 50)] # edit these for later
        upper_bound = np.array[(125, 255, 255)]

        try:

            # initialize variables ----------
            title_chars = [] # store title characters here
            clue_chars = [] # store clue characters here
            title = ""
            clue = ""

            lower_bound_sign = np.array([90, 50, 0])    # lower bound blue
            upper_bound_sign = np.array([120, 255, 190])   # upper bound blue
            lower_bound_letters = np.array([50, 50, 0])    # lower bound blue
            upper_bound_letters = np.array([255, 255, 255])   # upper bound blue

            # load image ----------
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # convert to HSV format
            height, width, _ = hsv.shape  # Get dimensions

            # crop out sky and ground ----------
            crop_start = height // 4  # remove top third of image
            crop_end = height - (height // 4) # remove bottom quarter of image
            cropped_image = hsv[crop_start:crop_end, :]  # crop

            # Apply gaussian blur ----------
            kernel_size = (5, 5)  # Adjust the kernel size to control the smoothing effect
            blurred_image = cv2.GaussianBlur(cropped_image, kernel_size, 0)

            # crop along outside of blue border ----------
            board_mask = cv2.inRange(blurred_image, lower_bound_sign, upper_bound_sign) # binary mask according to predefined bounds
            contours, _ = cv2.findContours(board_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # identify clue
            clue_border = max(contours, key=cv2.contourArea)  # Largest contour is the border of the sign
            x, y, w, h = cv2.boundingRect(clue_border)  # Get bounding box
            clue_sign = cropped_image[y:y+h, x:x+w] # crop image to clue sign

            # binary mask ----------
            sign_mask = cv2.inRange(clue_sign, lower_bound_letters, upper_bound_letters) # binary mask according to predefined bounds

            # find clue border contour
            inverted_image = cv2.bitwise_not(sign_mask) # now inner part is white
            contours, _ = cv2.findContours(inverted_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            clue_border = max(contours, key=cv2.contourArea)  # Largest white contour is border of sign

            # get rid of stuff outside clue border
            mask = np.full_like(inverted_image, 0)
            cv2.drawContours(mask, [clue_border], -1, 255, thickness=cv2.FILLED)
            result = cv2.bitwise_and(inverted_image, mask)

            # remove outer border ----------
            x, y, w, h = cv2.boundingRect(clue_border)  # Get bounding box of inner

            # compensate for warped perspectives ----------
            rect = np.zeros((4, 2), dtype="float32")
            cnt = clue_border.reshape(-1, 2)

            # Sorting the contour points
            s = cnt.sum(axis=1)
            rect[0] = cnt[np.argmin(s)]  # Top-left
            rect[2] = cnt[np.argmax(s)]  # Bottom-right
            diff = np.diff(cnt, axis=1)
            rect[1] = cnt[np.argmin(diff)]  # Top-right
            rect[3] = cnt[np.argmax(diff)]  # Bottom-left

            width, height = 300, 200  # Adjust this as needed
            dst = np.array([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]], dtype="float32")
            M = cv2.getPerspectiveTransform(rect, dst)
            warped_image = cv2.warpPerspective(inverted_image, M, (width, height))
            warped_image = warped_image[8:height-8, 8:width-8]

            # detect letter contours ----------
            reinvert_sign = cv2.bitwise_not(warped_image) # brings back to black background white letters

            split_point = reinvert_sign.shape[0] / 2  # reference for title clue split
            word_contours, _ = cv2.findContours(reinvert_sign, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # find groups of contours

            for cnt in word_contours: # for each word...

                x, y, w, h = cv2.boundingRect(cnt) # Draw bounding boxes around each word (contour)
                avg_ar = 1.7 # approximate height to width ratio of each letter
                num_let_approx = int(w * avg_ar / h) # approximate number of letters in word bounding box

                if num_let_approx == 0: # avoid division by zero
                    num_let_approx = 1

                letter_width = w / num_let_approx # approximate width of each letter

                # separate out each letter of the word
                for i in range(num_let_approx): # for each letter....
                    
                    # coordinates of each letter box
                    letter_x = x + i * letter_width
                    letter_y = y
                    letter_w = letter_width
                    letter_h = h

                    # Ensure all coordinates are integers
                    letter_x = int(letter_x)
                    letter_y = int(letter_y)
                    letter_w = int(letter_w)
                    letter_h = int(letter_h)

                    # save each letter
                    letter_img = reinvert_sign[letter_y:letter_y+letter_h, letter_x:letter_x+letter_w] # crop letter out of sign
                    letter_img = cv2.resize(letter_img, (28,28)) # reformat

                    # store in title or clue, depending on y coordinate
                    if letter_y < split_point:
                        title_chars.append(letter_img)
                    else:
                        clue_chars.append(letter_img)
                # if for letter
            # if for each word

            model = load_model("PATH TO MODEL HERE")
            characters = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ")  # Define list of possible characters

            for char in title_chars:
                char = char.reshape(1, *char.reshape) # add batch dimension
                pred_vector = model.predict(char)
                pred_label = characters[np.argmax(pred_vector)]  # Convert prediction to character
                title += pred_label

            for char in clue_chars:
                char = char.reshape(1, *char.reshape) # add batch dimension
                pred_vector = model.predict(char)
                pred_label = characters[np.argmax(pred_vector)]  # Convert prediction to character
                clue += pred_label

            # publish to score tracker
            self.score_pub.publish(f'fabs,password,1,{clue}')

        # # try
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        node = ClueReader()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass