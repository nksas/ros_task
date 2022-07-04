#!/usr/bin/env python3
 
# Import the necessary libraries
import rospy # Python library for ROS
from geometry_msgs.msg import Twist
import main_2

def publish_message():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
     
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('velocity_data_py', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz
  
  vel_msg = Twist()
  vel_msg.linear.x = 1

  # While ROS is still running.
  while not rospy.is_shutdown():
     
      
      rospy.loginfo('publishing velocity data')
            
      
      pub.publish(vel_msg)
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
