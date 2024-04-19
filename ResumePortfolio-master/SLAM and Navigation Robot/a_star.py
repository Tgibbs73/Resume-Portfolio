#!/usr/bin/env python

"""
a_star.py
Node for implementing the A* pathfinding algorithm

Subscribes to:
	/cspace
	/robot_pose
Publishes to:
	/frontier_gridcells
	/expanded_gridcells
	/path_gridcells
	/waypoint_gridcells
	/start_gridcell
	/goal_gridcells
	/timing_debug
	/a_star_debug
"""

import rospy
from PriorityQueue import *
from Stopwatch import *
from copy import deepcopy
from math import sqrt, atan2
from og_helper import *
from ros_util import *
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import GridCells, OccupancyGrid, Odometry
from std_msgs.msg import String, Float64
from lab4.srv import *

class a_star():

	def __init__(self):
		"""
		Brief: Constructor for A* node
		Inputs: None
		Return: None
		"""

		# ROS initializations
		rospy.init_node('a_star', anonymous = True)
		rospy.Subscriber('/cspace', OccupancyGrid, self.cspace_callback)
		rospy.Subscriber('/robot_pose', Pose, self.robot_pose_callback)
		self.waypoint_gc_pub = rospy.Publisher('/waypoint_gridcells', GridCells, queue_size = 1)
		self.timing_pub = rospy.Publisher('/timing_debug', String, queue_size = 1)
		self.debug_pub = rospy.Publisher('/a_star_debug', String, queue_size = 1)
		self.service = rospy.Service('waypoints', CalcWaypoints, self.a_star_callback)
		
		# Field definitions
		self.root2 = sqrt(2.0)
		self.cspace_radius_buffer = 0.1
		self.got_robot_pose = False
		self.got_cspace = False
		
		# C-space expansion offsets (used for alternate A* starting position)
		cspace_radius = rospy.get_param('~cspace_radius') + self.cspace_radius_buffer
		grid_size = rospy.get_param('~grid_size')
		self.cspace_offsets = og_gen_circular_offsets_ij(cspace_radius, grid_size)

		# Parameter extraction
		self.robot_drive_speed = rospy.get_param('~drive_speed')
		self.robot_turn_speed = rospy.get_param('~turn_speed')

	def cspace_callback(self, cspace):
		"""
		Brief: Copies C-space and determines gridcell expansion
		Inputs: cspace [OccupancyGrid]
		Return: None
		"""
		self.cspace = cspace
		self.got_cspace = True

	def robot_pose_callback(self, pose):
		"""
		Brief: Parses robot starting position
		Inputs: pose [Pose]
		Return: None
		"""
		self.start_x = pose.position.x
		self.start_y = pose.position.y
		self.got_robot_pose = True

	def a_star_callback(self, req):
		"""
		Brief: Performs A* from current robot position to requested goal
		Inputs: req.goal [PoseStamped]
		Return: waypoints_xy [PoseArray]
		"""

		# Start timing
		stopwatch = Stopwatch()

		# Wait for odom and cspace
		while not (self.got_robot_pose and self.got_cspace):
			pass
		
		# Convert start and goal to (i,j)
		goal = req.goal.pose.position
		start_i, start_j = og_xy_to_ij(self.start_x, self.start_y, self.cspace)
		goal_i, goal_j = og_xy_to_ij(goal.x, goal.y, self.cspace)
		start = (start_i, start_j)
		goal = (goal_i, goal_j)

		# Invalid start exception
		if og_get_ij(start_i, start_j, self.cspace) != 0:

			# Search for valid local starting coordinates
			for (i_off, j_off) in self.cspace_offsets:
				i_alt = start_i + i_off
				j_alt = start_j + j_off
				if og_get_ij(i_alt, j_alt, self.cspace):
					start_i = i_alt
					start_j = j_alt
					found_alt_start = True
					break

			# Publish warning or exception
			if found_alt_start:
				self.debug_pub.publish('Warning: Invalid start, rerouted')
			else:
				self.debug_pub.publish('Exception: Invalid start')
				return CalcWaypointsResponse(PoseArray(), Float64(0.0), String('start'))

		# Invalid goal exception
		if og_get_ij(goal_i, goal_j, self.cspace) != 0:
			self.debug_pub.publish('Exception: Invalid goal')
			return CalcWaypointsResponse(PoseArray(), Float64(0.0), String('goal'))
		
		# Run A* pathfinding
		frontier = PriorityQueue()
		frontier.put(start, 0)
		came_from = {}
		came_from[start] = None
		g_cost = {}
		g_cost[start] = 0

		while not frontier.empty():
			
			# Expand highest priority node
			curr = frontier.pop()
			if curr == goal:
				break

			# Generate neighbors
			i = curr[0]
			j = curr[1]
			neighbors = []
			for i_n in range(i - 1, i + 2):
				for j_n in range(j - 1, j + 2):
					if og_get_ij(i_n, j_n, self.cspace) == 0:
						neighbors.append((i_n, j_n))

			# Explore neighbors
			for next in neighbors:

				# Compute g-cost from curr
				new_g_cost = g_cost[curr] + self.edge_cost(curr, next)

				# Expand or improve frontier
				if next not in g_cost or new_g_cost < g_cost[next]:
					came_from[next] = curr					
					g_cost[next] = new_g_cost
					priority = g_cost[next] + self.h_cost(next, goal)
					frontier.put(next, priority)
	
		# No path found exception
		if curr != goal:
			self.debug_pub.publish('Exception: No path found')
			return CalcWaypointsResponse(PoseArray(), Float64(0.0), String('path'))

		# Construct waypoints from start to goal
		waypoints_ij = [goal]
		while True:
			first = came_from[waypoints_ij[0]]
			if first == start:
				break
			else:
				waypoints_ij.insert(0, first)
		
		# Convert waypoints from (i,j) to (x,y)
		waypoints_xy = PoseArray()
		cells = []
		for waypoint_ij in waypoints_ij:
			i = waypoint_ij[0]
			j = waypoint_ij[1]
			x, y = og_ij_to_xy(i, j, self.cspace)
			wp = Pose()
			wp.position = Point(x, y, 0.0)
			waypoints_xy.poses.append(wp)

		# Publish gridcell topics to RViz
		if req.pub_gridcells.data:
			og_pub_gridcells(waypoints_ij, 0.008, self.waypoint_gc_pub, self.cspace)

		# Finish timing
		dur = stopwatch.stop()
		timing_msg = 'A* pathfinding: ' + str(dur) + ' [s]'
		self.timing_pub.publish(timing_msg)

		# Return service response object
		self.debug_pub.publish('Path found!')
		time_cost = g_cost[goal]
		return CalcWaypointsResponse(waypoints_xy, Float64(time_cost), String('none'))

	def edge_cost(self, curr, next):
		"""
		Brief: Returns A* traversal cost between two 8-connected nodes
		Inputs:
			curr [(i, j)]: Current node
			next [(i, j)]: Next node
		Return: cost [float]: Edge traversal distance in grid cell counts
		"""
		
		# Parse (i,j) coordinates
		c_i = curr[0]
		c_j = curr[1]
		n_i = next[0]
		n_j = next[1]

		# Compute edge cost
		drive_dist = 1.0
		if (n_i != c_i) and (n_j != c_j):
			return self.root2
		else:
			return 1.0

	def h_cost(self, node, goal):
		"""
		Brief: Returns A* heuristic cost from node to goal
		Inputs:
			node [(i, j)]: Test node
			goal [(i, j)]: Goal node
		Return: cost [float]: Straight-line drive distance to goal in grid cell counts
		"""

		# Parse (i, j) coordinates
		n_i = node[0]
		n_j = node[1]
		g_i = goal[0]
		g_j = goal[1]

		# Compute straight-line drive distance
		return og_dist_ij(n_i, n_j, g_i, g_j)
		
if __name__ == '__main__':
	node = a_star()
	rospy.spin()
