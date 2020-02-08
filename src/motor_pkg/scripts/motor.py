#!/usr/bin/env python

import rospy
from motor_package.motor_controller import MotorController
from motor_package.diff_drive_adapter import DiffDriveAdapter

if __name__ == '__main__':
	rospy.init_node('motor')

	rospy.loginfo(rospy.get_name() + " Starting Motor Controller")
	myMotorController = MotorController()

	rospy.loginfo(rospy.get_name() + " Starting Diff Drive Adapter")
	diffDriveAdapter = DiffDriveAdapter(myMotorController)
	diffDriveAdapter.spin()
