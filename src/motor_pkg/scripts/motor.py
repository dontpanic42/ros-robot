#!/usr/bin/env python

import rospy
import motor_package.listener

if __name__ == '__main__':
	rospy.init_node('motor')
	motor_package.listener.listen()