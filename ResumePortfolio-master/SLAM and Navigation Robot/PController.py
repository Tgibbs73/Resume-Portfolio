"""
PController.py
Class for implementing limited proportional control
Implements u = p * error where u is limited to [cmd_min, cmd_max]
"""

from ros_util import *

class PController:

	def __init__(self, p_gain, cmd_min, cmd_max):
		"""
		Brief: Constructor for proportional controller
		Inputs:
			p_gain [cmd_units/error_units]: Proportional gain
			cmd_min [cmd_units]: Minimum output command
			cmd_max [cmd_units]: Maximum output command
		"""
		self.p_gain = p_gain
		self.cmd_min = cmd_min
		self.cmd_max = cmd_max

	def update(self, error):
		"""
		Brief: Calculates command based on given error
		Inputs: error [error_units]
		Return: command [cmd_units]
		"""
		return limit(self.p_gain * error, self.cmd_min, self.cmd_max)
