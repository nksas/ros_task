#!/usr/bin/env python3

# Import the necessary libraries
from cv2 import Canny
from matplotlib import image
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np # Numpy Library

def make_coordinates(image,line_parameters):
  slope, intercept = line_parameters
  #y1 = image.shape[0]
  #y2 = int(y1*(3/5))
  y1 = 300
  y2 = 200
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
  #print(left_fit_average, " ---- ", right_fit_average)
  left_line = make_coordinates(image,left_fit_average)
  right_line = make_coordinates(image, right_fit_average)
  return np.array([left_line,right_line])

def display_lines(image,lines):
  line_image = np.zeros_like(image)
  if lines is not None:
    for x1,y1,x2,y2 in lines:
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

def velocity_calculator(image, lines):
  velocity = Twist()

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

  left_slope = left_fit_average[0]
  right_slope = right_fit_average[0]
  
  if ((left_slope<-0.5 and left_slope>-0.8) and (right_slope>0.5 and right_slope<1)):
    velocity.linear.x = 0.5
    velocity.angular.z = 0
  else :
    velocity.linear.x = 0
    velocity.angular.z = 0

  return velocity

def publish_velocity(image,lines):
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  #rospy.init_node('velocity_data_py', anonymous=True)
  rate = rospy.Rate(10) # 10hz

  while not rospy.is_shutdown():
     
      
      rospy.loginfo('publishing velocity data')
            
      pub.publish(velocity_calculator(image,lines))
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()

def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
  
  lane_image = np.copy(current_frame)

  gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

  blur = cv2.GaussianBlur(gray, (5, 5), 0)

  canny_image = cv2.Canny(blur, 100, 200)

  cropped_image = region_of_interest(canny_image)
  
  lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=30, maxLineGap=15)

  averaged_lines = average_slope_intercept(lane_image, lines)

  #publish_velocity(lane_image,lines)

  line_image = display_lines(lane_image, averaged_lines)

  combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)

  cv2.imshow("camera", combo_image)

  #publish_velocity(lane_image,lines)
   
  cv2.waitKey(1)
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the camera feed gazebo topic
  rospy.Subscriber('/camera/color/image_raw', Image, callback)
  
  #publish_velocity()
  

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  try:
    receive_message()
    
  except rospy.ROSInterruptException:
    pass