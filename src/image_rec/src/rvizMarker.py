import roslib
import rospy
import numpy as np
import math
import cv2
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

class RvizMarker():
	def __init__(self, base_frame, marker_topic):
		self.base_frame = base_frame
		self.marker_topic = marker_topic
		# Set the default Marker parameters
 		self.setDefaultMarkerParams()

        	# Create the Rviz Marker Publisher
        	self.loadMarkerPublisher()

	def setDefaultMarkerParams(self):
		self.marker_lifetime = rospy.Duration(0.0) # 0 = Marker never expires
		self.alpha = 1.0


		# Sphere Marker (A single sphere)
		# This renders a low-quality sphere
		self.sphere_marker = Marker()
		self.sphere_marker.header.frame_id = self.base_frame
		self.sphere_marker.ns = "Sphere" # unique ID
		self.sphere_marker.type = Marker().SPHERE
		self.sphere_marker.action = Marker().ADD
		self.sphere_marker.lifetime = self.marker_lifetime
		self.sphere_marker.pose.position.x = 0
		self.sphere_marker.pose.position.y = 0
		self.sphere_marker.pose.position.z = 0
		self.sphere_marker.scale.x = 0.5
		self.sphere_marker.scale.y = 0.5
		self.sphere_marker.scale.z = 0.5
		self.sphere_marker.color.a = self.alpha
		self.sphere_marker.color.r = 0.0
		self.sphere_marker.color.g = 1.0
		self.sphere_marker.color.b = 1.0
		self.sphere_marker.pose.orientation.x = 0.0
		self.sphere_marker.pose.orientation.y = 0.0
		self.sphere_marker.pose.orientation.z = 0.0
		self.sphere_marker.pose.orientation.w = 1.0

	def loadMarkerPublisher(self):
		if hasattr(self, 'pub_rviz_marker'):
			return
        	# Create the Rviz Marker Publisher
		self.pub_rviz_marker = rospy.Publisher(self.marker_topic, Marker, queue_size=1)
		rospy.logdebug("Publishing Rviz markers on topic '%s'", self.marker_topic)

	def publishMarker(self, pose):
		self.loadMarkerPublisher()
		self.sphere_marker.pose = pose
		self.pub_rviz_marker.publish(self.sphere_marker)

		
	def getPose(self, x, y, w, h):
		center = np.array([(2*x + w)/2, (2*y + h)/2])

		focal_length = 0.5*512/(math.tan(math.pi/8))
		u0 = 512./2.
		v0 = 512./2.



		pose = Pose()
		pose.position.x = 256/(w * math.tan(math.pi/8))#(center[0]-u0) / focal_length
		pose.position.y = 256/(h * math.tan(math.pi/8)) #(center[1]-v0) / focal_length
		pose.position.z = 0
		im_pix_h = h / 2
		im_theta = ((math.pi / 8) * h) / (512)
		dist_pix = (im_pix_h) / math.tan(im_theta)
		dist_x_real = ((dist_pix * 0.5) / im_pix_h)
		pose.position.x = dist_x_real
		pose.position.y = ((x + w / 2) / 512)
		pose.position.z = 0

		#pose.position.x  = dist_x_real
		#pose.position.y = ((x + w / 2) / 512)
		#pose.position.z = 0


		pose.orientation.x = 0.0
		pose.orientation.y = 0.0
		pose.orientation.z = 0.0
		pose.orientation.w = 1.0
		return pose
