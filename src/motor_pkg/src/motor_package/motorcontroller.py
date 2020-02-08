import rospy
from motor_pkg.msg import motorcontroller_status
from motor_pkg.msg import motorcontroller_output
import pid

class MotorController:
	def __init__(self):
		self.last_data = None

		# Setup publisher to communicate with arduino
		self.output = rospy.Publisher(rospy.get_param("output_topic_name"), motorcontroller_output, queue_size=10)

		# Load gain parameters
		gains = rospy.get_param('gains')
		# Log gains
		rospy.loginfo(rospy.get_name() + " Initializing with (p=%s,i=%s,d=%s)", gains['P'], gains['I'], gains['D'])

		# Setup PID controller for left wheel
		self.lw_pid = pid.PID(gains['P'], gains['I'], gains['D'], sample_time=0.01)
		self.lw_pid.output_limits = (0, 255)

		# Setup PID controller for right wheel
		self.rw_pid = pid.PID(gains['P'], gains['I'], gains['D'], sample_time=0.01)
		self.rw_pid.output_limits = (0, 255)

		# Initial setup of enable left/right
		self.enable_left = False
		self.enable_right = False

		# Set initial speed
		self.set_desired_speed(0, 0)

		# Subscribe to the arduinos status updates
		rospy.Subscriber(rospy.get_param("status_topic_name"), motorcontroller_status, self.status_update)

	def set_desired_speed(self, lw=None, rw=None):
		if lw is not None:
			self.enable_left = (lw != 0)
			self.lw_pid.setpoint = float(lw)

		if rw is not None:
			self.enable_right = (rw != 0)
			self.rw_pid.setpoint = float(rw)

	def status_update(self, data):
		if self.last_data is None:
			self.last_data = data;
			return

		output = motorcontroller_output()
		output.lw_speed = 0
		output.rw_speed = 0

		# Get the milliseconds between the last message (as reported by the messages timestamp)
		# and the current message
		time_delta = float(data.stamp.to_nsec() - self.last_data.stamp.to_nsec()) / 1000.0 / 1000.0

		# Only proceed for the left wheel update when there was no tick overflow...
		if self.last_data.lw_ticks <= data.lw_ticks:

			# Calculate how many ticks occured between the last run an this run
			tick_delta = data.lw_ticks - self.last_data.lw_ticks
			# Convert the delta to ticks per ms
			tick_rate = float(tick_delta) / time_delta

			# Start the pid controller with our values
			output.lw_speed = self.lw_pid(tick_rate, dt=time_delta)

		if self.last_data.rw_ticks <= data.rw_ticks:

			# Calculate how many ticks occured between the last run an this run
			tick_delta = data.rw_ticks - self.last_data.rw_ticks
			# Convert the delta to ticks per ms
			tick_rate = float(tick_delta) / time_delta

			# Start the pid controller with our values
			output.rw_speed = self.rw_pid(tick_rate, dt=time_delta)

		self.last_data = data;
		self.output.publish(output)