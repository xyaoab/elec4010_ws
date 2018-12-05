#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from time import sleep
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

twist = Twist()
Xerror_prior = 0
Xintegral = 0
XKP = 0.05
XKI = 0
XKD = 0.5
Yerror_prior = 0
Yintegral = 0
YKP = 0.05
YKI = 0
YKD = 0.1
iteration_time=0.125
track_state = Bool()
track_state.data = False
switch_flag = False


class ball_tracker:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/vrep/image",Image,self.callback)
    self.state_sub = rospy.Subscriber("/track/bool",Bool,self.state)
    self.Twist_pub = rospy.Publisher('/vrep/cmd_vel',Twist,queue_size=5)

  def state(self,data):
      global track_state
      global switch_flag
      if data.data==True:
          switch_flag=True

      track_state.data = data.data;


  def callback(self,data):
    global Xerror_prior
    global Xintegral
    global XKP
    global XKI
    global XKD
    global Yerror_prior
    global Yintegral
    global YKP
    global YKI
    global YKD
    global iteration_time
    global track_state
    global switch_flag
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    cv_image = cv2.flip(cv_image,1)
    (rows,cols,channels) = cv_image.shape
    cv2.imshow("Image window", cv_image)

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_yellow = np.array([20,100,100])
    upper_yellow = np.array([30,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

    cv2.imshow('frame',cv_image)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

    moments = cv2.moments(mask)
    m00 = moments['m00']
    centroid_x, centroid_y = None, None
    if m00 != 0:
        centroid_x = int(moments['m10']/m00)
        centroid_y = int(moments['m01']/m00)

    # Assume no centroid
    ctr = (255,350)
    #cv2.circle(cv_image, ctr, 10, (255,0,0))
    # Use centroid if it exists
    twist.linear.x = 0
    twist.angular.z = 0
    if switch_flag==True :
        twist.angular.z = 0
        twist.linear.x = 0
        self.Twist_pub.publish(twist)
        switch_flag = False
        
    if centroid_x != None and centroid_y != None and track_state.data == True:


        ctr = (centroid_x, centroid_y)

        # Put black circle in at centroid in image
        cv2.circle(cv_image, ctr, 10, (0,0,0))

        Xerror = 255 - centroid_x
        Xintegral = Xintegral + (Xerror*iteration_time)
        Xderivative = (Xerror - Xerror_prior)/iteration_time
        twist.angular.z = XKP*Xerror + XKI*Xintegral + XKD*Xderivative
        Xerror_prior = Xerror

        Yerror = 350 - centroid_y
        Yintegral = Yintegral + (Yerror*iteration_time)
        Yderivative = (Yerror - Yerror_prior)/iteration_time
        twist.linear.x = YKP*Yerror + YKI*Yintegral + YKD*Yderivative
        Yerror_prior = Yerror
        self.Twist_pub.publish(twist)
    # Display full-color image
    cv2.imshow("mark", cv_image)

    k = cv2.waitKey(5) & 0xFF


def main(args):
  ic = ball_tracker()
  twist.linear.x = 0
  twist.angular.z = 0
  rospy.init_node('ball_tracker', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

