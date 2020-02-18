#!/usr/bin/env python
'''
Created on Monday, January 6, 2020
@author: Albert Go
Client code for the button box; takes data from arduino serial monitor
'''

import serial
import actionlib
import re
import rospy
from button_box.srv import *
from std_msgs.msg import (
	Bool,
	String,
	Int32,
	Float64,
	Float64MultiArray,
	UInt16,
	Empty
)
from sawyer.msg import *

class ButtonClient():
	def __init__(self):
		rospy.init_node('client', anonymous=True)

		self.pressed = False
		self.button  = None

		#Publisher topics
		self.button_topic = rospy.Publisher('/teachbot/button', String, queue_size=10)

		#Subscribe and listen to any action requests from the button topic
		self.ButtonPressAct = actionlib.SimpleActionServer('/teachbot/ButtonSend', ButtonSendAction, execute_cb=self.cb_buttonPress, auto_start=True)

		#Connect to the arduino serial port
		self.arduino = serial.Serial("/dev/ttyACM0", 9600)
		rospy.loginfo('Connected to Arduino')

		#Constantly listen for any buttons that are pressed throughout
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			button_raw = str(self.arduino.readline())

			#Split the serial line readout and take the first item
			self.button = re.split(": |\r|\n", button_raw)[0]
			if self.button != 'start' and self.button != 'On' and self.button != 'Off':
				rospy.loginfo('Publishing...')
				self.button_topic.publish(self.button)
			if self.button != "start":
				self.pressed = True
				self.send_buttonInfo()

			r.sleep()

	def cb_buttonPress(self, goal):
		'''
		Action server callback function that receives a goal from the module.js action client
		The result should be a button being pressed and it returns True
		If there is an error, it will automaticallt return False
		'''
		result_press = ButtonSendResult()
		rospy.loginfo('Awaiting button press')

		go = True
		while go:
			#Make sure that a button is pressed before continuing
			if self.button != 'start' and self.button != None and self.pressed:
				self.send_buttonInfo()
				go = False

		#Tell the action client that the user has pressed a button
		result_press.done = True
		self.ButtonPressAct.set_succeeded(result_press)

	def send_buttonInfo(self):
		'''
		Define the client and send data to server
		Await for a response before moving on
		'''
		rospy.loginfo('sending' + ' ' + self.button)
		rospy.wait_for_service('/teachbot/buttons')
		try:
			buttons = rospy.ServiceProxy('/teachbot/buttons', ButtonInfo)
			send = buttons(self.button)
			rospy.loginfo(send.response)
			self.pressed = False
			return send.response

		except rospy.ServiceException, e:
			rospy.loginfo('Service did not process request:'%e)


if __name__ == '__main__':
	ButtonClient()
	rospy.spin()

