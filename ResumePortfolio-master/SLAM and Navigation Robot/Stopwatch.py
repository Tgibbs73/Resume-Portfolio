"""
Stopwatch.py
Brief: Class for measuring timing of computations in ROS
"""

import rospy

class Stopwatch:

	def __init__(self):
		"""
		Brief: Constructs new stopwatch and resets
		Inputs: None
		Return: None
		"""
		self.reset()

	def reset(self):
		"""
		Brief: Resets stopwatch
		Inputs: None
		Return: None
		"""
		self.t_tic = rospy.Time.now().to_sec()

	def stop(self):
		"""
		Brief: Returns time in seconds since last reset
		Inputs: None
		Return: [float]
		"""
		t_toc = rospy.Time.now().to_sec()
		dur = t_toc - self.t_tic
		return dur

	def has_elapsed(self, dur):
		"""
		Brief: Returns true if stopwatch has elapsed given duration
		Inputs: dur [float]: Duration in seconds
		Return: Boolean
		"""
		return self.stop() > dur
