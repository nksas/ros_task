#!/usr/bin/env python3

# Import the necessary libraries
from cv2 import Canny
from matplotlib import image
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np # Numpy Library
import matplotlib.pyplot as plt # Importing the pyplot package from the matplotlib library

def make_coordinates(image,line_parameters):
  slope, intercept = line_parameters
  #y1 = image.shape[0]
  #y2 = int(y1*(3/5))
  y1 = 350
  y2 = 150
  x1 = int((y1 - intercept)/slope)
  x2 = int((y2 - intercept)/slope)
  return np.array([x1,y1,x2,y2])


def average_slope_intercept(image, lines):
  left_fit = []
  right_fit = []
  for line in lines:
    x1,y1,x2,y2 = line.reshape(4)
    parameters = np.polyfit((x1,x2), (y1,y2), 1)
    slope = parameters[0]
    intercept = parameters[1]
    if slope < 0:
      left_fit.append((slope, intercept))
    else:
      right_fit.append((slope,intercept))
  left_fit_average = np.average(left_fit, axis=0)
  right_fit_average = np.average(right_fit, axis=0)
  left_line = make_coordinates(image,left_fit_average)
  right_line = make_coordinates(image, right_fit_average)
  return np.array([left_line,right_line])

def display_lines(image,lines):
  line_image = np.zeros_like(image)
  if lines is not None:
    for line in lines:
      x1,y1,x2,y2 = line.reshape(4)
      cv2.line(line_image, (x1,y1), (x2,y2), (0,0,255), 10)
  return line_image


def region_of_interest(image):
  height = image.shape[0]
  width = image.shape[1]
  square = np.array([[(0,280),(360,130),(width,280)]])
  mask = np.zeros_like(image)
  cv2.fillPoly(mask, square, 255)
  masked_image = cv2.bitwise_and(image, mask)
  return masked_image


def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
  
  # Creating a numpy array that is a copy of the image feed
  lane_image = np.copy(current_frame)

  # Converting image to grayscale
  gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

  # Applying a Gaussian Blur
  blur = cv2.GaussianBlur(gray, (5, 5), 0)

  # Edge Detection using the Canny Edge Detector
  canny_image = cv2.Canny(blur, 100, 200)

  # Cropped image
  cropped_image = region_of_interest(canny_image)
  
  lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=30, maxLineGap=15)

  averaged_lines = average_slope_intercept(lane_image, lines)

  line_image = display_lines(lane_image, averaged_lines)

  combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)

  # Display image using cv2
  cv2.imshow("camera", combo_image)
   
  cv2.waitKey(1)

  # Display image using matplotlib
  #plt.imshow(edges)

  #plt.show()
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the camera feed gazebo topic
  rospy.Subscriber('/camera/color/image_raw', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()