#!/usr/bin/env python

"""
waypoint_driver.py
Node for driving robot through a series of waypoints.

Subscribes to:
	/robot_pose
	/move_base_simple/goal
Publishes to:
	/cmd_vel
	/waypoint_driver_debug
"""

import rospy
from PController import *
from Stopwatch import *
from math import atan2, hypot, pi
from ros_util import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import Bool, String
from lab4.srv import *

class waypoint_driver:

	def __init__(self):
		"""
		Brief: Initializes and runs driver node
		Inputs: None
		Return: None
		"""
		
		# ROS initializations
		rospy.init_node('waypoint_driver', anonymous = True)
		rospy.Subscriber('/robot_pose', Pose, self.robot_pose_callback)
		rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
		self.debug_pub = rospy.Publisher('/waypoint_driver_debug', String, queue_size = 1)

		# Robot pose
		self.pose_x = 0.0	# Robot x [m]
		self.pose_y = 0.0	# Robot y [m]
		self.pose_h = 0.0	# Robot heading [rad]
		
		# Nav controller constants
		self.ctrl_rate = 30.0	# Control rate [Hz] (was 30)
		self.h_gain = 1.2		# Heading proportional gain [(rad/s)/rad]
		self.d_max = 0.05		# Goal distance threshold [m]
		self.v_max = rospy.get_param('~drive_speed')
		self.w_max = rospy.get_param('~turn_speed')
		self.cspace_radius = rospy.get_param('~cspace_radius')

		# Nav controller objects
		self.h_controller = PController(self.h_gain, -self.w_max, self.w_max)

		# Velocity commands
		self.cmd_v = 0.0	# Linear velocity [m/s]
		self.cmd_w = 0.0	# Angular velocity [rad/s]

		# Navigation state initialization
		self.state = 'waiting'	# Navigation state [String]
		self.wp_count = 0		# Number of waypoints [n]
		self.wp_index = 0		# Current waypoint index [0, n-1]
		self.wp_x = 0.0			# Current waypoint x [m]
		self.wp_y = 0.0			# Current waypoint y [m]
		self.new_goal = False	# New goal received flag [Boolean]
			
		# Constants
		twopi = 2.0 * pi
		invpi = 1.0 / pi

		# Navigation state machine
		self.rate = rospy.Rate(self.ctrl_rate)
		while not rospy.is_shutdown():
			if self.state == 'waiting':
				
				# Stop the robot and wait
				self.vel_pub.publish(Twist())

				# State changes
				if self.new_goal:
					self.state = 'newgoal'

			elif self.state == 'newgoal':

				# Reset waypoint navigation state
				self.new_goal = False
				self.wp_count = len(self.waypoints)
				goal = self.waypoints[self.wp_count - 1]
				self.goal_x = goal.position.x
				self.goal_y = goal.position.y
				self.wp_index = 0

				# State changes
				if self.wp_count > 0:
					self.set_waypoint()
					self.state = 'navigating'
				else:
					self.state = 'waiting'

			elif self.state == 'navigating':
				
				# Calculate position error
				wp_error_x = self.wp_x - self.pose_x
				wp_error_y = self.wp_y - self.pose_y
				waypoint_dist = hypot(wp_error_y, wp_error_x)
				goal_error_x = self.goal_x - self.pose_x
				goal_error_y = self.goal_y - self.pose_y
				goal_dist = hypot(goal_error_y, goal_error_x)

				# Point towards waypoint heading with heading proportional control
				waypoint_h = atan2(wp_error_y, wp_error_x)
				h_error = mod_angle_error(waypoint_h - self.pose_h)
				self.cmd_w = self.h_controller.update(h_error)

				# Limit top speed based on angular error
				h_control_factor = max(0.0, 1.0 - 2.0 * abs(h_error * invpi))
				self.cmd_v = self.v_max * h_control_factor

				# Publish command at limited rate
				cmd_vel = Twist()
				cmd_vel.linear.x = self.cmd_v
				cmd_vel.angular.z = self.cmd_w
				self.vel_pub.publish(cmd_vel)

				# State changes
				if self.new_goal:
					self.state = 'newgoal'
				elif goal_dist < self.cspace_radius:
					self.debug_pub.publish('Reached goal!')
					self.state = 'waiting'
				elif waypoint_dist < self.d_max:
					self.debug_pub.publish('Reached waypoint!')
					self.wp_index += 1
					if self.wp_index == self.wp_count:
						self.state = 'waiting'
					else:
						self.set_waypoint()

			# Limit control rate
			self.rate.sleep()

	def set_waypoint(self):
		"""
		Brief: Calculates waypoint (x, y) based on current waypont index
		Inputs: None
		Return: None
		"""
		self.wp_x, self.wp_y, h = pose_to_xyh(self.waypoints[self.wp_index])

	def robot_pose_callback(self, pose):
		"""
		Brief: Parses robot pose x, y, and heading
		Inputs: pose [Pose]
		Return: None
		"""
		self.pose_x, self.pose_y, self.pose_h = pose_to_xyh(pose)

	def goal_callback(self, goal):
		"""
		Brief: Requests A* path to given goal
		Inputs: goal [PoseStamped]
		Return: None
		"""

		# Request waypoints from A*
		rospy.wait_for_service('waypoints')
		get_waypoints = rospy.ServiceProxy('waypoints', CalcWaypoints)
		resp = get_waypoints(goal, Bool(True))
		if resp.exception.data == 'none':
			self.debug_pub.publish('Valid goal received!')
			self.waypoints = resp.waypoints.poses
			self.new_goal = True
		else:
			self.debug_pub.publish('Invalid goal received!')
			self.new_goal = False
			
if __name__ == '__main__':
	node = waypoint_driver()
	rospy.spin()
