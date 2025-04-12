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

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

# Imports for Machine Learning
from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers
from tensorflow.keras import callbacks
from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend

from Levenshtein import distance

class CompetitionRobot:
    """
    @class PathFollower
    @brief Uses camera inputs to steer robot in line following path
    """
    def __init__(self):
        """
        @brief Constructor for PathFollower
        """
        rospy.init_node('competition_robot', anonymous=True) # change in your launch file!
        self.controller_pkg_path = '/home/fizzer/ENPH353_comp/ros_ws/src/controller_pkg/src/'
        self.model_path = os.path.join(self.controller_pkg_path, 'model/best_model_ever.keras')
        print(f"LOADED MODEL: {self.model_path}")
        # Load the model trained in Colab
        self.top_cutoff_ratio = 0.33
        self.conv_model = models.load_model(self.model_path)
        self.action_dict = {0: [0.0,0.0], 1:[0.5,0.0], 2:[0.0,-2.0], 3:[0.0, 2.0]}

        self.clue_mod = models.load_model(os.path.join(self.controller_pkg_path, 'clue_model/model(5).h5'))

        self.teamname = 'fabs'
        self.password = '9'

        # variables to compare when checking if signs of frames are to be read
        self.prev_clue = ""
        self.prev_title = ""
        self.title_read_count = 0

        self.bridge = CvBridge()
        self.twist = Twist()
        self.atTunnel = False
        self.stopCount = 0

        rospy.sleep(5)

        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=10)

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/B1/pi_camera/image_raw', Image, self.image_callback)

        # Publisher for score tracking
        self.score_pub = rospy.Publisher('/score_tracker', String, queue_size=1)

        rospy.sleep(2)
        
        self.score_pub.publish('fabs,nopassword,0,NA')
        
        self.timer_initialized = False
        self.time_stopped = False
        self.timeout = 0
        self.TIMEOUT_THRESHOLD = 10
        
        
        self.move()

        
        

    def image_callback(self, msg):
        """
        @brief  A callback function to handle new incoming images. Converts ROS Image formats to OpenCV,
                calls the processing function to handle path finding and velocity adjustments. Also publishes a 
                processed image for debugging purposes (shows a window of the camera's pov)
        @param  msg contains image data
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # # Convert ROS Image message to OpenCV format
            if self.atTunnel == False:
                self.search_signs(cv_image)
            else:
                self.velocity = self.get_velocity(cv_image)

            # Send image to clue reading NN/processing
            # TODO: Integrate code for reading clue boards
            # self.search_signs(cv_image)
            # Send image to driving NN/processing
            # if self.atTunnel:
            #     self.velocity = self.get_velocity(cv_image) # want get_velocity to output the an array size 2, [linear vel, angular vel]
            
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", str(e))
    
    def move(self):
        ############ ROAD #######################
        # self.score_pub.publish('fabs,nopassword,0,NA')
        # initial full send


        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.vel_pub.publish(self.twist)
        rospy.sleep(0.1)

        self.spawn_position([5.4994102600397605, 2.5031470486024037, 0.04000033327085088, 0, 0, -0.709216019930764, 0.7049912319119358])

        rospy.sleep(3)

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

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(0.45)

        self.twist.linear.x = 0.0
        self.twist.angular.z = -2
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.459)

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
        self.twist.linear.x = -2
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(0.6)

        self.twist.linear.x = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(3.0)

        self.twist.linear.x = 2
        self.vel_pub.publish(self.twist)
        rospy.sleep(0.6)

        self.twist.linear.x = 0
        self.vel_pub.publish(self.twist)

        rospy.sleep(0.1)

        ######## NEW ################
        self.twist.linear.x = 1
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(9.3)

        self.twist.linear.x = 1.1
        self.twist.angular.z = 2.25
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.5)

        self.twist.linear.x = 1
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(6.5)

        self.twist.linear.x = 0
        self.twist.angular.z = 2
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.8)

        self.twist.linear.x = 1
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(5.7)

        self.twist.linear.x = 0
        self.twist.angular.z = 2
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.8)

        self.twist.linear.x = 1
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(4.8)

        self.twist.linear.x = 0
        self.twist.angular.z = 2
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.8)

        self.twist.linear.x = 2
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(1.5)

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(0.6)

        ###############################
        self.score_pub.publish('fabs,nopassword,-1,NA')
        # self.atTunnel = True
        
        

    def search_signs(self, cv_image):
        """ 
        @brief Process the image to detect the clue board and output strings 
        @param cv_image contains image data in ROS image format
        @return String message containing clue data
        @return processed_image a NumPy array of the binarized image, for display and debugging purposes
        """

        try:

            # initialize variables ----------
            title_chars = [] # store title characters here
            clue_chars = [] # store clue characters here
            sorted_title_chars = []
            sorted_clue_chars = []
            title = ""
            clue = ""

            lower_bound_sign = np.array([0, 115, 0])    # lower bound blue
            upper_bound_sign = np.array([20, 255, 215])   # upper bound blue
            lower_bound_letters = np.array([0, 100, 0])    # lower bound blue
            upper_bound_letters = np.array([20, 255, 255])   # upper bound blue

            # load image ----------
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV) # convert to HSV format
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
                        title_chars.append([letter_img, letter_x])
                    else:
                        clue_chars.append([letter_img, letter_x])

                    # sort left to right
                    sorted_title_chars =  [img for img, x in sorted(title_chars, key=lambda item: item[1])]
                    sorted_clue_chars = [img for img, x in sorted(clue_chars, key=lambda item: item[1])]
                # if for letter
            # if for each word

            characters = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")  # Define list of possible characters

            # predict characters (title and clue)
            for char in sorted_title_chars:
                char = char.reshape(1, *char.shape) # add batch dimension
                pred_vector = self.clue_mod.predict(char, verbose=0)
                pred_label = characters[np.argmax(pred_vector)]  # Convert prediction to character
                title += pred_label

            for char in sorted_clue_chars:
                char = char.reshape(1, *char.shape) # add batch dimension
                pred_vector = self.clue_mod.predict(char, verbose=0)
                pred_label = characters[np.argmax(pred_vector)]  # Convert prediction to character
                clue += pred_label
            
            # self.score_pub.publish(f'fabs,password,{title},{clue}')

            # check if valid clue
            if title == self.prev_title and clue == self.prev_clue and clue != "":# if title and clue are same as previous
                self.title_read_count += 1
                if self.title_read_count >= -1:
                    clue_num = self.which_clue(title)
                    if clue_num != None:
                        # code to publish to score tracker
                        self.score_pub.publish(f'fabs,password,{clue_num},{clue}')
                        self.title_read_count = 0
            else:
                self.title_read_count = 0

            self.prev_title = title
            self.prev_clue = clue
                    
            # publish to score tracker
            # self.score_pub.publish(f'fabs,password,{clue_num},{clue}')

        # # try
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def which_clue(self, det_title):

        clues = ["SIZE", "VICTIM", "CRIME", "TIME", "PLACE", "MOTIVE", "WEAPON", "BANDIT"]
        
        accuracy_arr = [distance(det_title, clue) for clue in clues]

        if any(a is None for a in accuracy_arr):  # Safety check
            raise ValueError("distance() returned None for some values.")

        if min(accuracy_arr) < 2:
            clue_index = accuracy_arr.index(min(accuracy_arr))  # Find best match
            return clue_index + 1
        else:
            return None
    def get_velocity(self, cv_image):
        """
            @brief  Get velocity command based on the inputted 1200 x 1500 RGB image

            @param  cv_image - a 1200 x 1500 (should work with any similar aspect ratioed image, work better for larger than smaller)
                    cv2 image object
            @return action_vector[0] - predicted linear velocity
            @return action_vector[1] - predicted angular velocity

        """
        # get process image to put into NN
        processed_img = self.process_img_driving(cv_image)

        # compute prediction
        prediction = self.conv_model.predict(np.expand_dims(processed_img, axis=0), verbose=0)[0]
        # print(prediction)

        # recover action based on prediction
        action_key = max(enumerate(prediction), key=lambda x: x[1])[0]
        if action_key == 0:
            self.stopCount += 1
        else:
            self.stopCount = 0
        if self.stopCount >= 20:
            self.atTunnel = False
        
        action_vector = self.action_dict[action_key]
        self.twist.linear.x = action_vector[0]
        self.twist.angular.z = action_vector[1]
        # if np.random.rand() < 0.2:
        #     self.twist.angular.z = 2
        # Publish movement command
        self.vel_pub.publish(self.twist)

        # Telemetry!
        resized_img = cv2.resize(cv_image, (500,400), interpolation= cv2.INTER_LINEAR)
        self.image_height = resized_img.shape[0]
        self.image_width = resized_img.shape[1]
        cv2.putText(resized_img, f'Action Taken: {action_vector}', (int(self.image_width - self.image_width*0.9), int(self.image_height*0.1)), 
                        cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255))
        cv2.imshow("Original Image", resized_img)
        cv2.waitKey(1)
        return

    def process_img_driving(self, cv_image):
        """
            @brief  Takes in colour image outputs Numpy array of grayscale,
                    blurred, and resized image
            
            @param  cv_image - cv2 colour image object with aspect ration comparable to 1200 x 1500
            @return processed image to get rid of noise and resized to make putting into NN easier
        """
        # gray scale
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        # blur that bitch
        blur_img = cv2.GaussianBlur(gray_img, (15,15), 0)
        # resize to 100 x 80
        img_np = np.array(blur_img)
        img_np_resized = cv2.resize(img_np, (100, 80))
        height_upper_cutoff = int(img_np_resized.shape[0]*self.top_cutoff_ratio)
        img_np_resized = img_np_resized[:][height_upper_cutoff:]
        cv2.imshow("Processed Image", img_np_resized)
        cv2.waitKey(1)
        return img_np_resized
    def process_image(self, cv_image):
        """ 
            @brief  Process the image to detect the path and compute velocity commands 

            @param  cam_raw contains image data in ROS image format
            @return velocity Twist message containing data for movement adjusted for current and desired trajectory
            @return processed_image a NumPy array of the binarized image, for display and debugging purposes
        """
        try:
            # self.score_pub.publish('fabs,password,0,NA')
            # Convert the ROS Image message to an OpenCV image
            # cv_image = self.bridge.imgmsg_to_cv2(cam_raw, desired_encoding="bgr8")
            if self.timer_initialized == False:
                rospy.sleep(1)
                self.score_pub.publish('fabs,password,0,NA')
                self.timer_initialized = True
                
                
            
            height = cv_image.shape[0]
            width = cv_image.shape[1]
            blurred = cv2.blur(cv_image, (12, 12))
            # Convert to grayscale
            hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
            road_lr = np.array([0, 0, 65])
            road_ur = np.array([0, 0, 90])
            mask = cv2.inRange(hsv, road_lr, road_ur)
            # Show the processed image

            row_of_interest = height - 100  # Line 75 pixels from the bottom
            line_data = mask[row_of_interest, 0:width]  # Extract the specific row
            right_found = False
            left_found = False

            leftmost = 0
            rightmost = width - 1

            for x in range(len(line_data)):
                # Get the leftmost and rightmost points
                if not left_found and line_data[x] == 255:
                    leftmost = x
                    left_found = True
                if not right_found and line_data[width - 2 - x] == 255:
                    rightmost = width - 1 - x
                    right_found = True
            if right_found or left_found:
                self.timeout = 0
                # Calculate the midpoint
                midpoint = (leftmost + rightmost) // 2

                # Find how far off we are from being centered
                error = midpoint - width/2

                rate = rospy.Rate(2)
                self.twist.linear.x = 0.5
                self.twist.angular.z = -error * 0.008

                # Publish movement command
                self.vel_pub.publish(self.twist)

                cv2.circle(cv_image, (int(midpoint), row_of_interest), 20, (255,0,0), -1)
            else:
                self.vel_pub.publish(self.twist)
                self.timeout += 1
                cv2.circle(cv_image, (int(width/2), row_of_interest), 20, (255,0,0), -1)
                if self.timeout >= self.TIMEOUT_THRESHOLD and self.time_stopped == False:
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    self.vel_pub.publish(self.twist)
                    self.score_pub.publish('fabs,password,-1,NA')
                    self.time_stopped = True
            cv2.imshow("White Line", cv_image)
            cv2.waitKey(1)

            cv2.imshow("Mask", mask)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

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
        node = CompetitionRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass