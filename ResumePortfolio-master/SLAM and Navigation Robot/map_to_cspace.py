#!/usr/bin/env python

"""
map_to_cspace.py
Node for converting map to more restrictive C-space

Subscribes to:
	/map
Publishes to:
	/cspace
	/timing_debug
"""

import rospy
from Stopwatch import *
from copy import deepcopy
from math import ceil
from og_helper import *
from ros_util import *
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

class map_to_cspace:

	def __init__(self):
		"""
		Brief: Initializes node
		Inputs: None
		Return: None
		"""

		# ROS initializations
		rospy.init_node('map_to_cspace')
		rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
		self.cspace_pub = rospy.Publisher('/cspace', OccupancyGrid, queue_size = 1)
		self.timing_pub = rospy.Publisher('/timing_debug', String, queue_size = 1)

		# Generate C-sapace expansion offsets
		cspace_radius = rospy.get_param('~cspace_radius')
		grid_size = rospy.get_param('~grid_size')
		self.expansion_offsets = og_gen_circular_offsets_ij(cspace_radius, grid_size)

	def map_callback(self, map_og):
		"""
		Brief: Converts given map to cspace and publishes to /cspace
		Inputs: map_og [OccupancyGrid]
		Return: None
		"""

		# Start timing
		stopwatch = Stopwatch()
		
		# Generate C-space from map
		cspace_og = deepcopy(map_og)
		cspace_og.data = list(cspace_og.data)
		m = map_og.info.height
		n = map_og.info.width
		for i in range(m):
			for j in range(n):
				if og_get_ij(i, j, map_og) >= 50:
					for (i_off, j_off) in self.expansion_offsets:
						og_set_ij(i + i_off, j + j_off, 100, cspace_og)
		cspace_og.data = tuple(cspace_og.data)

		# Publish C-space grid
		self.cspace_pub.publish(cspace_og)

		# Finish timing
		dur = stopwatch.stop()
		timing_msg = 'Map to C-space: ' + str(dur) + ' [s]'
		self.timing_pub.publish(timing_msg)

if __name__ == '__main__':
	node = map_to_cspace()
	rospy.spin()
