#!/usr/bin/env python
'''
Create on Monday, January 6, 2020
@author: Albert Go
Test server code to the button client code
'''
from button_box.srv import *
import rospy

def get_buttonInfo(req):
	rospy.loginfo(req.button)
	if req.button == 'Blue':
		return 'You picked blue'
	elif req.button == "Red":
		return 'You picked red'
	elif req.button == "Black":
		return 'You picked black'
	elif req.button == 'Yellow':
		return 'You picked yellow'
	elif req.button == 'WhiteL':
		return 'You picked white left'
	elif req.button == 'WhiteR':
		return 'You picked white right'
	elif req.button == "On":
		return 'Skip features enabled'
	elif req.button == "Off":
		return 'Skip features disabled'

def button_server():
	rospy.init_node('buttons')
	s = rospy.Service('/teachbot/buttons', ButtonInfo, get_buttonInfo)
	rospy.spin()

if __name__ == '__main__':
	button_server()