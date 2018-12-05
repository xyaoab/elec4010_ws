#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import roslib
import os
import sys
import rospy
import math
import cv2
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from rvizMarker import RvizMarker
from squares import find_squares,  get_side

templates =['pic001.jpg', 'pic002.jpg', 'pic003.jpg', 'pic004.jpg', 'pic005.jpg']
templates_path = sys.path[0]+'/picture/'

face_names = {
  templates_path+templates[0] : "Obama",
  templates_path+templates[1] : "Arvil",
  templates_path+templates[2] : "GEGE",
  templates_path+templates[3] : "Legolas",
  templates_path+templates[4] : "levi"
}
cv_flag = True
cv_window_flag = False

class image_converter:

  def __init__(self):
    self.laser_scan_index = 450
    self.laser_data = 0.
    self.image_pub = rospy.Publisher("/vrep/imagecov",Image,queue_size=1)
    self.bridge = CvBridge()
    self.cv_switch_sub = rospy.Subscriber("/cv_switch/bool",Bool,self.cv_switch)
    self.image_sub = rospy.Subscriber("/vrep/image",Image,self.callback)
    self.laser_sub = rospy.Subscriber("/vrep/scan", LaserScan, self.laser_callback)


  def cv_switch(self,switch_cv):
      global cv_flag
      cv_flag = switch_cv.data

  def laser_callback(self, scan_data):
    if(scan_data.ranges[self.laser_scan_index]):
      self.laser_data = scan_data.ranges[self.laser_scan_index]


  def callback(self,data):
      global cv_flag
      global cv_window_flag
      if cv_flag == True:

          try:
              cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
              cv_image = cv2.flip(cv_image,1)

          except CvBridgeError as e:
            print(e)
          ##### detect face
          ## init rviz marker node
          self.marker = RvizMarker('base_link', '/rviz/marker')
          self.marker.setDefaultMarkerParams()

          face = self.face_detect(cv_image)

          cv2.imshow("Image window", face)
          cv_window_flag = True
          cv2.waitKey(1)

          try:
              self.image_pub.publish(self.bridge.cv2_to_imgmsg(face, "bgr8"))
          except CvBridgeError as e:
              print(e)
      else:
          if cv_window_flag == True:
              cv2.destroyAllWindows()
              cv_window_flag = False
          print("Face recognition turned off !!")
  # human face
  def face_detect(self,img):
    self.good_match=[]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    faceCascade = cv2.CascadeClassifier(sys.path[0]+'/haarcascade_frontalface_default.xml')
    #faceCascade = cv2.CascadeClassifier('/home/xinjie/elec4010_ws/src/image_rec/src/haarcascade_frontalface_default.xml')
    faces = faceCascade.detectMultiScale(gray,scaleFactor=1.12,
      minNeighbors=5,minSize=(30, 30))
    print ("Found {0} human faces!".format(len(faces)))
    #human face detected
    if len(faces) != 0:
      for (x,y,w,h) in faces:

        squares = find_squares(img)
        cv2.drawContours( img, squares, 0, (120,0,120), 3 )
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)

        #publish pose in rviz

        pose = self.estimate_depth(x, y, w, h)
        self.marker.publishMarker(pose)

      for i in range(4):
        self.template_matching(templates_path+templates[i], img, False, True)
      cv2.putText(img, face_names[templates_path+templates[np.argmax(self.good_match)]],
          (50, 150),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),2)
      print(self.good_match)

    # search for levi
    else:
      img = self.template_matching(templates_path+templates[4], img, True, False)
    return img

  #anime face
  def template_matching(self, template_path, img, draw=False, recog=False):

    template = cv2.imread(template_path)

    sift =  cv2.xfeatures2d.SIFT_create()
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(template,None)
    kp2, des2 = sift.detectAndCompute(img,None)
    # FLANN parameters
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)   # or pass empty dictionary
    flann = cv2.FlannBasedMatcher(index_params,search_params)
    if(len(kp1)>=2 and len(kp2)>=2) :
      matches = flann.knnMatch(np.asarray(des1, np.float32),np.asarray(des2,np.float32),k=2)
    else:
      matches = []
    # Need to draw only good matches, so create a mask
    good = []
    # ratio test as per Lowe's paper
    for (m,n) in matches:
        if m.distance < .7*n.distance:
            good.append(m)
    if recog:
        self.good_match.append([len(good)])

    if len(good)>6:
      src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
      dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
      # draw bounding box

      if draw:

        squares = find_squares(img)
        cv2.drawContours( img, squares, 0, (120,0,120), 3 )
        #publish pose in rviz
        if squares:
          xx,yy,ww,hh = get_side(squares)
          pose = self.estimate_depth(xx, yy, ww, hh)
          self.marker.publishMarker(pose)

        cv2.putText(img, face_names[template_path], (50, 150),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),2)

    else:
      print ("NO FACE /n Not enough matches are found - %d/%d" % (len(good),8))
      matchesMask = None

    return img


  def estimate_depth(self,x,y,w,h):
    center = np.array([256-(2*x + w)/2, 512-(2*y + h)/2])

    self.laser_scan_index = 450 - center[0]/2
    angle_diff = math.atan(float(center[0]) / 256 * math.tan(22.5 * math.pi / 180))

    x = self.laser_data * math.sin(angle_diff)
    y = -self.laser_data * math.cos(angle_diff)
    print("3d loc", x,y)
    pose = self.marker.getPose(x,y)
    return pose



def main(args):
  print(sys.path[0])
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
