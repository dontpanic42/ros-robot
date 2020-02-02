#!/user/bin/env python

import rospy
from std_msgs.msg import Int16

def talk():
	pub_lw_speed = rospy.Publisher('lw_speed', Int16, queue_size=10)
	pub_rw_speed = rospy.Publisher('rw_speed', Int16, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		lw_speed = 45
		rw_speed = 45

		rospy.loginfo("%s Sending speed %s (lw), %s (rw)", rospy.get_name(), lw_speed, rw_speed)
		pub_lw_speed.publish(lw_speed)
		pub_rw_speed.publish(rw_speed)
		rate.sleep()