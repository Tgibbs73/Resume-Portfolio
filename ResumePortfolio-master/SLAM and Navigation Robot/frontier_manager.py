#!/usr/bin/env python

"""
frontier_manager.py
Node for determining and prioritizing frontiers in C-space

Subscribes to:
	/cspace
	/robot_pose
Publishes to:
	/frontier_gc
	/centroids_gc
	/move_base_simple/goal
	/timing_debug
	/frontier_debug
"""

import rospy
from Frontier import *
from PriorityQueue import *
from Queue import *
from Stopwatch import *
from og_helper import *
from ros_util import *
from geometry_msgs.msg import Point, Pose, PoseStamped
from math import hypot
from nav_msgs.msg import GridCells, OccupancyGrid, Odometry
from std_msgs.msg import String, Bool
from lab4.srv import *

class frontier_manager():

	def __init__(self):
		"""
		Brief: Initializes ROS node
		Inputs: None
		Returns: None
		"""

		# Field definitions
		self.robot_initial_x = 0.0
		self.robot_initial_y = 0.0
		self.robot_x = 0.0
		self.robot_y = 0.0
		self.got_initial_pose = False
		self.nav_state = 'exploring'
		
		# ROS initializations
		rospy.init_node('frontier_manager')
		rospy.Subscriber('/cspace', OccupancyGrid, self.cspace_callback)
		rospy.Subscriber('/robot_pose', Pose, self.robot_pose_callback)
		self.pub_frontiers_gc = rospy.Publisher('/frontiers_gc', GridCells, queue_size = 1)
		self.pub_centroids_gc = rospy.Publisher('/centroids_gc', GridCells, queue_size = 1)
		self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
		self.pub_timing = rospy.Publisher('/timing_debug', String, queue_size = 1)
		self.pub_debug = rospy.Publisher('/frontier_debug', String, queue_size = 1)

	def robot_pose_callback(self, pose):
		"""
		Brief: Parses robot position from odom
		Inputs: odom [Odometry]
		"""
		if not self.got_initial_pose:
			self.robot_initial_x = pose.position.x
			self.robot_initial_y = pose.position.y
			self.got_initial_pose = True
		self.robot_x = pose.position.x
		self.robot_y = pose.position.y

	def cspace_callback(self, cspace_og):
		"""
		Brief: Callback for /cspace
		Details:
			- Constructs list of frontiers from C-space
			- Publishes frontier and centroid gridcells
			- Publishes goal position of highest priority frontier centroid
		Inputs: cspace_og [OccupancyGrid]
		Return: None
		"""

		# Start timing
		stopwatch = Stopwatch()

		# Build frontier list from C-space
		frontier_list = []
		m = cspace_og.info.height
		n = cspace_og.info.width
		for i in range(m):
			for j in range(n):
				if og_on_frontier_ij(i, j, cspace_og):

					# Potential frontier starting point
					start = (i, j)

					# Check against existing frontiers
					new_frontier = True
					for frontier in frontier_list:
						if frontier.has_point(start):
							new_frontier = False
							break
					if new_frontier:

						# Generate new frontier from starting point
						frontier = Frontier(start)
	
						# Expand using wavefront algorithm
						queue = Queue()
						queue.put(start)
						visited = {}
						while not queue.empty():
							curr = queue.pop()
							visited[curr] = True
							i_c = curr[0]
							j_c = curr[1]
							for i_n in range(i_c - 1, i_c + 2):
								for j_n in range(j_c - 1, j_c + 2):
									next_on_frontier = og_on_frontier_ij(i_n, j_n, cspace_og)
									next = (i_n, j_n)
									if (next not in visited) and next_on_frontier:
										visited[next] = True
										queue.put(next)
										frontier.add_point(next)

						# Add to list of frontiers
						frontier_list.append(frontier)
		
		# Convert frontier list to priority queue
		frontier_points = []
		centroid_points = []
		frontier_queue = PriorityQueue()
		for frontier in frontier_list:

			# Goal position is centroid of frontier
			centroid = frontier.get_centroid()
			c_i = centroid[0]
			c_j = centroid[1]
			c_x, c_y = og_ij_to_xy(c_i, c_j, cspace_og)
			goal = PoseStamped()
			goal.pose.position = Point(c_x, c_y, 0.0)
	
			# Get path time cost from A*
			rospy.wait_for_service('waypoints')
			get_waypoints = rospy.ServiceProxy('waypoints', CalcWaypoints)
			resp = get_waypoints(goal, Bool(False))

			# If valid path exists:
			if resp.exception.data == 'none':

				# Add frontier to priority queue
				priority = resp.time_cost.data
				frontier_queue.put(frontier, priority)

				# Add points to gridcells
				frontier_points += frontier.get_points()
				centroid_points.append(frontier.get_centroid())

		# Check if map is complete
		if frontier_queue.empty():

			# Map complete - publish home position to goal
			if self.nav_state == 'exploring':
				goal = PoseStamped()
				goal.pose.position = Point(self.robot_initial_x, self.robot_initial_y, 0.0)
				self.pub_goal.publish(goal)
				self.pub_debug.publish('Map complete - returning home')
				self.nav_state = 'returning'
				if hypot(
					self.robot_initial_x - self.robot_x,
					self.robot_initial_y - self.robot_y) < 0.2:
					self.pub_debug.publish('Returned home - ready to race')
					self.nav_state = 'racing'

		else:

			# Map incomplete - publish top priority frontier to goal
			if self.nav_state != 'racing':
				i, j = frontier_queue.pop().get_centroid()
				x, y = og_ij_to_xy(i, j, cspace_og)
				goal = PoseStamped()
				goal.pose.position = Point(x, y, 0.0)
				self.pub_goal.publish(goal)
				self.pub_debug.publish('Frontier goal published')
				self.nav_state = 'exploring'

		# Publish frontier and centroid gridcells
		og_pub_gridcells(frontier_points, 0.002, self.pub_frontiers_gc, cspace_og)
		og_pub_gridcells(centroid_points, 0.004, self.pub_centroids_gc, cspace_og)

		# Finish timing
		dur = stopwatch.stop()
		timing_msg = 'Frontier generation: ' + str(dur) + ' sec'
		self.pub_timing.publish(timing_msg)

if __name__ == '__main__':
	node = frontier_manager()
	rospy.spin()
