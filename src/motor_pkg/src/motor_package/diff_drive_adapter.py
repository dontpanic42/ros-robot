import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32

class DiffDriveAdapter:
	def __init__(self, mc):
		self.mc = mc

		self.lwheel_rate_pub = rospy.Publisher("/lwheel_rate", Float32, queue_size=10)
		self.rwheel_rate_pub = rospy.Publisher("/rwheel_rate", Float32, queue_size=10)

		self.lwheel_ticks_pub = rospy.Publisher("/lwheel_ticks", Int32, queue_size=10)
		self.rwheel_ticks_pub = rospy.Publisher("/rwheel_ticks", Int32, queue_size=10)

		rospy.Subscriber("/lwheel_desired_rate", Int32, self.lwheel_desired_rate_callback)
		rospy.Subscriber("/rwheel_desired_rate", Int32, self.rwheel_desired_rate_callback)

	# Callback for "lwheel_desired_rate" topic message
	def lwheel_desired_rate_callback(self, msg):
		# Note: diff_drive publishes the rate in ticks per second. The motor controller
		# class works with ticks per millisecond, we need to convert the values
		rate = 0.0 if (msg.data == 0) else (float(msg.data) / 1000)
		self.mc.set_desired_speed(lw=rate)

	# Callback for "rwheel_desired_rate" topic message
	def rwheel_desired_rate_callback(self, msg):
		# Note: diff_drive publishes the rate in ticks per second. The motor controller
		# class works with ticks per millisecond, we need to convert the values
		rate = 0.0 if (msg.data == 0) else (float(msg.data) / 1000)
		self.mc.set_desired_speed(rw=rate)

	def spin(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():

			lw_rate, rw_rate = self.mc.get_current_tickrate()
			lw_tick, rw_tick = self.mc.get_current_ticks()

			# Note: diff_drive expects the rate in ticks per second. The motor controller
			# class works with ticks per millisecond, we need to convert the values
			self.lwheel_rate_pub.publish(lw_rate * 1000.0)
			self.rwheel_rate_pub.publish(rw_rate * 1000.0)

			self.lwheel_ticks_pub.publish(lw_tick)
			self.rwheel_ticks_pub.publish(rw_tick)

			rate.sleep()


