import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Int16
import PID
import constant



def run():

	lw_controller = MotorController(constant.LW_ENCODER_TICKS_TOPIC_NAME, constant.LW_SPEED_TOPIC_NAME)
	# rw_controller = MotorController(constant.RW_ENCODER_TICKS_TOPIC_NAME, constant.RW_SPEED_TOPIC_NAME)

	rospy.loginfo(rospy.get_name() + " Starting Motor Controller")
	rospy.spin()

class MotorController:
	def __init__(self, ticksTopicName, speedTopicName):
		self.ticks_last = 0
		self.time_last = self.current_time()

		self.speed_pub = rospy.Publisher(speedTopicName, Int16, queue_size=10)

		self.pid = PID.PID(100, 200, 5, setpoint=2.0, sample_time=0.01)
		self.pid.output_limits = (0, 255)

		rospy.Subscriber(ticksTopicName, Int32, self.ticks_callback)

	def current_time(self):
		return rospy.get_rostime().to_nsec() / 1000 / 1000

	def ticks_callback(self, data):
		ticks_delta = data.data - self.ticks_last
		self.ticks_last = data.data

		# The ticks delta should never be negative, if it is, we had an overflow
		# and we need to skip this cycle
		if(ticks_delta < 0):
			return


		time_delta = self.current_time() - self.time_last
		self.time_last = self.current_time()
		ticks_rate = (float(ticks_delta) / float(time_delta)) if (ticks_delta != 0.0 and time_delta != 0) else 0.0 


		output = self.pid(ticks_rate)
		rospy.loginfo(rospy.get_caller_id() + " rate %s, time delta %s, tick delta %s, out %s", ticks_rate, time_delta, ticks_delta, output)

		self.speed_pub.publish(output)