#!/usr/bin/env python

import rospy
import motor_package.motorcontroller

if __name__ == '__main__':
	rospy.init_node('motor')
	motor_package.motorcontroller.run()