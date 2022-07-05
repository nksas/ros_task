#!/usr/bin/env python3
 
# Import the necessary libraries
import rospy # Python library for ROS
from geometry_msgs.msg import Twist

def velocity_calculator():
  vel_msg = Twist()
  vel_msg.linear.x = 1
  return vel_msg


def publish_message():
  
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  rospy.init_node('velocity_data_py', anonymous=True)
  rate = rospy.Rate(10) # 10hz

  while not rospy.is_shutdown():
     
      
      rospy.loginfo('publishing velocity data')
            
      pub.publish(velocity_calculator())
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()


if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
