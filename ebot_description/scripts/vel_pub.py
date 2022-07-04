#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


def increase(Twist):

  print('We are in the callback function!')
  velo_msg = Twist
  
  # l = Twist.linear.x
  # print(l)
  # define the rate at which
  # we will be publishing the velocity.
  rate = rospy.Rate(5)

  # prompt the user for the acceleration value
  speed = float(input('By how much would \
  you like to accelerate? '))

  while not rospy.is_shutdown():
    
    # increase the current velocity by the speed defined.
    velo_msg.linear.x = (Twist.linear.x) + 2
    
    # publish the increased velocity
    pub.publish(velo_msg)
    print('Publishing was successful!')
    rate.sleep()


def main():
  print("In main...")
  
  # initializing the publisher node
  rospy.init_node('Velocity_publisher', anonymous=True)
  sub = rospy.Subscriber('/cmd_vel', Twist, increase)
  rospy.spin()


if __name__ == '__main__':
  try:
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    main()
  except rospy.ROSInterruptException:
    pass
