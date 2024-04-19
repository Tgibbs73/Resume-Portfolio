#!/usr/bin/env python

"""
odom_to_map.py
Node for transforming robot /odom pose to /map pose

Publises to:
	/robot_pose
	/odom_to_map_debug
"""

import rospy
import tf
from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_multiply
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class odom_to_map:
	
	def __init__(self):
		"""
		Brief: Initializes and runs node
		Inputs: None
		Return: None
		"""
		
		# ROS Initializations
		rospy.init_node('odom_to_map', anonymous = True)
		listener = tf.TransformListener()
		pose_pub = rospy.Publisher('/robot_pose', Pose, queue_size = 1)
		debug_pub = rospy.Publisher('/odom_to_map_debug', String, queue_size = 1)
		rate = rospy.Rate(30.0)

		# Publish robot pose repeatedly
		while not rospy.is_shutdown():
	
			# Get transform
			try:
				(pos, quat) = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
			except:
				debug_pub.publish('Transform exception')
				continue

			# Parse and publish pose
			robot_pose = Pose()
			robot_pose.position.x = pos[0]
			robot_pose.position.y = pos[1]
			robot_pose.position.z = pos[2]
			robot_pose.orientation.x = quat[0]
			robot_pose.orientation.y = quat[1]
			robot_pose.orientation.z = quat[2]
			robot_pose.orientation.w = quat[3]
			pose_pub.publish(robot_pose)

			# Limit publish rate
			rate.sleep()

if __name__ == '__main__':
	node = odom_to_map()
	rospy.spin()
