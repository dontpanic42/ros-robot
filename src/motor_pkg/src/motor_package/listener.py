import rospy
from std_msgs.msg import String

def listener_callback(data):
	rospy.loginfo(rospy.get_caller_id() + " Got data %s", data.data)

def listen():
	rospy.loginfo(rospy.get_name() + " Starting listener")
	rospy.Subscriber("chatter", String, listener_callback)
	rospy.spin()
