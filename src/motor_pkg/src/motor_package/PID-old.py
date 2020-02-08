#!/usr/bin/python
# 
# Generic pid controller, based on the code found here:
# https://raw.githubusercontent.com/ivmech/ivPID/master/PID.py
#
# How to use: Initialize after node initialization, set target
# with PidController.SetPoint = (float)
#

import rospy

class PidController:

	def __init__(self, P=0.2, I=0.0, D=0.0):

		self.Kp = P
		self.Ki = I
		self.Kd = D

		self.sample_time = rospy.Duration(nsecs=1 * 1000 * 1000)
		self.current_time = rospy.get_rostime()
		self.last_time = rospy.get_rostime()

		self.reset()

	def reset(self):

		self.SetPoint = 0.0
		self.PTerm = 0.0
		self.ITerm = 0.0
		self.DTerm = 0.0
		self.last_error = 0.0

		# Windup Guard
		self.windup_guard = 20.0

		self.output = 0.0

	def update(self, feedback_value):
		"""Calculates PID value for given reference feedback

		.. math::
			u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

		.. figure:: images/pid_1.png
		   :align:   center

		   Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
		"""

		error = self.SetPoint - feedback_value;

		self.current_time = rospy.get_rostime()
		delta_time = self.current_time - self.last_time
		delta_time_ms = delta_time.to_nsec() / 1000 / 1000
		delta_error = error - self.last_error

		if (delta_time > self.sample_time):
			self.PTerm = self.Kp * error
			self.ITerm += error * delta_time_ms

			if (self.ITerm < -self.windup_guard):
				self.ITerm = -self.windup_guard
			elif (self.ITerm > self.windup_guard):
				self.ITerm = self.windup_guard

			self.DTerm = 0.0
			if delta_time_ms > 0:
				self.DTerm = (delta_error / delta_time_ms) if delta_error != 0 else 0

			self.last_time = self.current_time
			self.last_error = error
			self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

			rospy.loginfo(rospy.get_caller_id() + " dt: %s, error: %s, delta_error: %s, output: %s", delta_time_ms, error, delta_error, self.output);

	def setKp(self, proportional_gain):
		"""Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
		self.Kp = proportional_gain

	def setKi(self, integral_gain):
		"""Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
		self.Ki = integral_gain

	def setKd(self, derivative_gain):
		"""Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
		self.Kd = derivative_gain

	def setWindup(self, windup):
		"""Integral windup, also known as integrator windup or reset windup,
		refers to the situation in a PID feedback controller where
		a large change in setpoint occurs (say a positive change)
		and the integral terms accumulates a significant error
		during the rise (windup), thus overshooting and continuing
		to increase as this accumulated error is unwound
		(offset by errors in the other direction).
		The specific problem is the excess overshooting.
		"""
		self.windup_guard = windup

	def setSampleTime(self, sample_time):
		"""PID that should be updated at a regular interval.
		Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
		"""
		self.sample_time = sample_time


