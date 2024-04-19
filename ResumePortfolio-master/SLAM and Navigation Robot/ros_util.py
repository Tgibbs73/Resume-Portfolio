"""
ros_util.py
General functions for use in Python and ROS
"""

import rospy
import sys
from math import pi
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

# Globals
twopi = 2.0 * pi

def limit(val, val_min, val_max):
    """
    Brief: Returns val constrained to range [val_min, val_max]
	Inputs:
		val [float]: Value to be limited
		val_min [float]: Minimum allowable value
		val_max [float]: Maximum allowable value
    Return: Limited val [float]
    """
    if val > val_max:
        return val_max
    elif val < val_min:
        return val_min
    else:
        return val

def mod_angle_error(error):
	"""
	Brief: Returns given angular error modulo constrained to [-pi, pi]
	Inputs: error [rad]: Raw angular error
	Return: error [rad]: Adjusted angular error
	"""
	if error >= pi:
		error -= twopi
	elif error <= -pi:
		error += twopi
	return error

def pose_to_xyh(pose):
	"""
	Brief: Converts pose to x, y, heading
	Inputs: pose [Pose]
	Return:
		x [float]: x-position [m]
		y [float]: y-position [m]
		h [float]: heading [rad]
	"""

	# Parse (x, y) from position
	x = pose.position.x
	y = pose.position.y

	# Parse heading from orientation
	q = pose.orientation
	quat = [q.x, q.y, q.z, q.w]
	r, p, h = euler_from_quaternion(quat)

	# Return values
	return x, y, h
