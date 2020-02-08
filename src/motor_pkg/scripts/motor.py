#!/usr/bin/env python

import rospy
from motor_package.motorcontroller import MotorController

if __name__ == '__main__':
	rospy.init_node('motor')
	rospy.loginfo(rospy.get_name() + " Starting Motor Controller")

	myMotorController = MotorController()

	rospy.spin()