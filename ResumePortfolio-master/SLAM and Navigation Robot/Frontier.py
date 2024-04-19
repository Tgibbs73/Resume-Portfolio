"""
Frontier.py
Brief: Class for implementing frontier clusters for ROS occupancy grids
Details: A frontier is a set of 8-connectedt adjacent points (i, j) in
an occupancy grid.
"""

from og_helper import *

class Frontier:

	def __init__(self, new_point):
		"""
		Brief: Constructs frontier of single point
		Inputs: new_point [tuple(i, j)]: First point in frontier
		Return: None
		"""
		self.points = [new_point]

	def add_point(self, new_point):
		"""
		Brief: Adds point to frontier (if non-duplicate)
		Inputs: new_point [tuple (i, j)]
		"""
		self.points.append(new_point)

	def has_point(self, point):
		"""
		Brief: Returns true if frontier includes given point
		Inputs: point [tuple(i, j)]
		Return: [Boolean]
		"""
		return (point in self.points)
	
	def get_points(self):
		"""
		Brief: Returns list of points (i, j) in the frontier
		Inputs: None
		Return: points [list(tuple(i, j))]
		"""
		return self.points
	
	def get_centroid(self):
		"""
		Brief: Returns point (i, j) in frontier closest to centroid
		Inputs: None
		Return: centroid [tuple(i, j)]
		"""
		
		# Calculate true centroid (c_i, c_j)
		len_inv = 1.0 / len(self.points)
		c_i = 0.0
		c_j = 0.0
		for (i, j) in self.points:
			c_i += i
			c_j += j
		c_i = c_i * len_inv
		c_j = c_j * len_inv

		# Find point in frontier (p_i, p_j) closest to centroid
		p_i = self.points[0][0]
		p_j = self.points[0][1]
		dist_min = og_dist_ij(p_i, p_j, c_i, c_j)
		for (i, j) in self.points:
			dist_new = og_dist_ij(i, j, c_i, c_j)
			if dist_new < dist_min:
				dist_min = dist_new
				p_i = i
				p_j = j
		
		# Return the point
		return (p_i, p_j)
		
