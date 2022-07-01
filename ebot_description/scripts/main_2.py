#!/usr/bin/env python3

# Import the necessary libraries
from cv2 import Canny
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np # Numpy Library
import matplotlib.pyplot as plt # Importing the pyplot package from the matplotlib library
 
def region_of_interest(image):
  height = image.shape[0]
  width = image.shape[1]
  square = np.array([[(0,280),(360,130),(width,280)]])
  mask = np.zeros_like(image)
  cv2.fillPoly(mask, square, 255)
  return mask


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
  gray = cv2.cvtColor(lane_image, cv2.COLOR_BGR2GRAY)

  # Applying a Gaussian Blur
  blur = cv2.GaussianBlur(gray, (5, 5), 0)

  # Edge Detection using the Canny Edge Detector
  canny = cv2.Canny(blur, 100, 200)


  
  # Display image using cv2
  cv2.imshow("camera", region_of_interest(canny))
   
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