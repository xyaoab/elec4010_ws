#!/usr/bin/env python
from __future__ import division
import roslib
import sys
import rospy
import random
import numpy as np
from time import sleep
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

twist = Twist()
exp_flag = False
count_flag = 0
rotate_count = 0
random.seed(4)
angle = random.randint(1,21)

class auto_explorer:

  def __init__(self):
    self.laser_sub = rospy.Subscriber("/vrep/scan",LaserScan,self.callback)
    self.state_sub = rospy.Subscriber("/exp_switch/bool",Bool,self.state_callback)
    self.Twist_pub = rospy.Publisher('/vrep/cmd_vel',Twist,queue_size=5)
  
  def state_callback(self,data):
      global exp_flag
      exp_flag = data.data


  def callback(self,data):

    global exp_flag
    global count_flag
    global rotate_count
    global angle
    min_distance = min(data.ranges[300:600])
    print("min distance is:" + str(min_distance))    #for debug
    print("rotate_count is:" + str(rotate_count))
    print("count_flag is:" + str(count_flag))
    if exp_flag == True:
        if rotate_count > 0:
            twist.linear.x = 0
            twist.angular.z = 0.5-angle/21
            self.Twist_pub.publish(twist)
            rotate_count -= 1

        elif count_flag > 1:
            count_flag = 0
            rotate_count = 130
            angle = random.randint(1,21)

        elif min_distance < 0.5:
            count_flag+=1

        else:
            twist.linear.x = 0.5
            twist.angular.z = 0
            self.Twist_pub.publish(twist)

   

def main(args):
  ic = auto_explorer()
  twist.linear.x = 0
  twist.angular.z = 0
  rospy.init_node('auto_explorer', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
