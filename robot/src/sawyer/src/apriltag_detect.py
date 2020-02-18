#! /usr/bin/env python

## IMPORTS ##
# Basic
import rospy, math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import apriltag

# Intera
import intera_interface
from std_msgs.msg import (
	Bool,
	String,
	Int32,
	Float64,
	Float64MultiArray,
	UInt16,
	Empty
)
import sensor_msgs
from sensor_msgs.msg import Image
import intera_core_msgs

# Custom
from limb_plus import LimbPlus
from image import Produce
from sawyer.msg import *

class Camera():
	
	## INITIALIZE ROS AND SAWYER ##
	def __init__(self):
		# Initialize Sawyer
		rospy.init_node('camera', anonymous=True)
	
		# Publishing topics
		suppress_cuff_interaction = rospy.Publisher('/robot/limb/right/suppress_cuff_interaction', Empty, queue_size=1)
		self.image_topic = rospy.Publisher('/image_rect', Image)

		self.limb = LimbPlus()
		self.produce = Produce()
		self.heights = []
		self.widths = []

		#Camera pos 1
		# self.limb.go_to_joint_angles([0.5832646484375, -1.301193359375, -0.193248046875, 2.0165146484375, 0.0075908203125, -0.75755078125, 0.3351982421875])
		#Camera pos 2
		self.limb.go_to_joint_angles([0.21234375, 0.2708505859375, -1.70441796875, 1.7846484375, 1.9352724609375, -0.17507421875, -3.083966796875])
		# rospy.sleep(3)

		self.start()

		# Initialization complete. Spin.
		rospy.loginfo('Ready.')
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			suppress_cuff_interaction.publish()
			r.sleep()

	def display_camera_callback(self, img_data): 
		bridge = CvBridge()
		# rospy.loginfo(img_data.height)
		try:
			cv_image = bridge.imgmsg_to_cv2(img_data, 'bgr8')
			# self.image_topic.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
		except CvBridgeError, err:
			rospy.logerr(err)

		try:
			img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
			detector = apriltag.Detector()
			detections = detector.detect(img)
			# rospy.loginfo(detections[0])
			centerApril = detections[0].center

			cv2.line(cv_image, (int(detections[0].corners[0][0]),int(detections[0].corners[0][1])), (int(detections[0].corners[1][0]),int(detections[0].corners[1][1])), (255,255,51), 3)
			cv2.line(cv_image, (int(detections[0].corners[1][0]),int(detections[0].corners[1][1])), (int(detections[0].corners[2][0]),int(detections[0].corners[2][1])), (255,255,51), 3)
			cv2.line(cv_image, (int(detections[0].corners[3][0]),int(detections[0].corners[3][1])), (int(detections[0].corners[2][0]),int(detections[0].corners[2][1])), (255,255,51), 3)
			cv2.line(cv_image, (int(detections[0].corners[3][0]),int(detections[0].corners[3][1])), (int(detections[0].corners[0][0]),int(detections[0].corners[0][1])), (255,255,51), 3)
		except:
			rospy.loginfo('No apriltag detected')
			pass

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5,5), 0)
		ret, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)

		im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

		cnt = contours[0]
		max_area = cv2.contourArea(cnt)

		for contour in contours:
			if cv2.contourArea(contour) > max_area:
				cnt = contour
				max_area = cv2.contourArea(contour)

		rect = cv2.minAreaRect(cnt)
		box = cv2.boxPoints(rect)
		box = np.int0(box)

		cv2.drawContours(cv_image, [box], -1, (0,255,0), 3)

		a = sum(map(lambda x: x[0], box))/4
		b = sum(map(lambda x: x[1], box))/4
		centerBin = (a,b)

		try:
			distance = ((centerBin[0]-centerApril[0])**2+(centerBin[1]-centerApril[1])**2)**0.5

			if distance < 56:
				rospy.loginfo('yay!')
			else: 
				rospy.loginfo(distance)
		except:
			rospy.loginfo('sad')
			pass
		# cv2.circle(cv_image, (a,b), 8, (255,0,0), -1)

		cv2.imwrite('/home/albertgo/TeachBot/browser/public/images/test.png', cv_image)

		cv2.imshow('new', cv_image)

		# cv2.imshow('binary', thresh)
		
		cv2.waitKey(5)

	def start(self):
		rp = intera_interface.RobotParams()
		valid_cameras = rp.get_camera_names()
		camera = intera_interface.Cameras()
		if not camera.verify_camera_exists('right_hand_camera'):
			rospy.logerr('Invalid camera name')
		camera.start_streaming('right_hand_camera')
		rectify_image = False
		camera.set_callback('right_hand_camera', self.display_camera_callback)

	def clean_shutdown(self):
		rospy.loginfo('Shutting down')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	Camera()