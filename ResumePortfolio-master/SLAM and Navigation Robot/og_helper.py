"""
og_helper.py
Functions for interfacing with ROS OccupancyGrid

Details:
- Grid coordinates (i, j) refer to discrete row-column indeces of grid
- Map coordinates (x, y) refer to continuous positions on the map
- OccupancyGrid arguments are abbreviated 'og'
"""

import rospy
from math import floor, hypot, ceil
from geometry_msgs.msg import Point
from nav_msgs.msg import  GridCells, OccupancyGrid

def og_valid_ij(i, j, og):
	"""
	Brief: Returns true if grid coordinates (i,j) exist inside og
	Inputs:
		i [int]
		j [int]
		og [OccupancyGrid]
	Return: Boolean
	"""
	return (i >= 0) and (i < og.info.height) and (j >= 0) and (j < og.info.width)

def og_get_ij(i, j, og):
	"""
	Brief: Returns og value at grid (i, j) or -1 for invalid coordinates
	Inputs:
		i [int]
		j [int]
		og [OccupancyGrid]
	Return: Value [int]: -1 to 100
	"""
	if og_valid_ij(i, j, og):
		return og.data[og.info.width * i + j]
	else:
		return -1

def og_on_frontier_ij(i, j, og):
	"""
	Brief: Returns true if index (i, j) of given og is on frontier
	Inputs:
		i [int]
		j [int]
		og [OccupancyGrid]
	Return: [Boolean]
	"""
	if og_get_ij(i, j, og) == 0:
		for i_n in range(i - 1, i + 2):
			for j_n in range(j - 1, j + 2):
				if og_valid_ij(i_n, j_n, og) and (og_get_ij(i_n, j_n, og) == -1):
					return True
	return False

def og_edit(og):
	"""
	Brief: Converts og data to list so it can be edited
	Inputs: og [OccupancyGrid]
	Return: None
	"""
	og.data = list(og.data)

def og_set_ij(i, j, val, og):
	"""
	Brief: Sets og value at (i, j) to val
	Inputs:
		i [int]
		j [int]
		val [int]
		og [OccupancyGrid]
	Return: None
	"""
	if og_valid_ij(i, j, og):
		og.data[og.info.width * i + j] = val

def og_save(og):
	"""
	Brief: Converts og data to tuple so it can be published
	Inputs: og [OccupancyGrid]
	return: None
	"""
	og.data = tuple(og.data)

def og_dist_ij(i1, j1, i2, j2):
	"""
	Brief: Returns pythagorean distance in grid counts between (i1, j1) and (i2, j2)
	Inputs:
		i1 [int]
		j1 [int]
		i2 [int]
		j2 [int]
	Return: distance [float]
	"""
	return hypot(i2 - i1, j2 - j1)

def og_gen_circular_offsets_ij(radius_xy, grid_size):
	"""
	Brief: Returns list of (i, j) offsets within radius using given grid size
	Inputs:
		radius_xy [float]: Radius in meters
		grid_size [float]: Grid size in meters
	Return: Offsets [list(tuple(i, j))]
	"""
	radius_ij = int(ceil(radius_xy / grid_size))
	offsets_ij = []
	for i in range(-radius_ij, radius_ij + 1):
		for j in range(-radius_ij, radius_ij + 1):
			if hypot(i, j) <= radius_ij:
				offsets_ij.append((i, j))
	return offsets_ij

def og_xy_to_ij(x, y, og):
	"""
	Brief: Determines discrete (i,j) grid of og containing point (x, y)
	Inputs:
		x [float]
		y [float]
		og [OcupancyGrid]
	Return:
		i [int]
		j [int]
	"""
	i = int(floor((y - og.info.origin.position.y) / og.info.resolution))
	j = int(floor((x - og.info.origin.position.x) / og.info.resolution))
	return i, j
	
def og_ij_to_xy(i, j, og):
	"""
	Brief: Determines map (x, y) coordinates at center of grid (i, j) in og
	Inputs:
		i [int]
		j [int]
		og [OcupancyGrid]
	Return:
		x [float]
		y [float]
	"""
	x = (j + 0.5) * og.info.resolution + og.info.origin.position.x
	y = (i + 0.5) * og.info.resolution + og.info.origin.position.y
	return x, y

def og_round_xy(x, y, og):
	"""
	Brief: Rounds (x, y) position to nearest grid center in og
	Inputs:
		x [float]: Unrounded
		y [float]: Unrounded
		og [OcupancyGrid]
	Return:
		x [float]: Rounded
		y [float]: Rounded
	"""
	# SAME OUTPUT NAME MIGHT BREAK IT???
	x = (floor(x / og.info.resolution) + 0.5) * og.info.resolution
	y = (floor(y / og.info.resolution) + 0.5) * og.info.resolution
	return x, y

def og_pub_gridcells(points, z, pub, og):
	"""
	Brief: Publishes list of points (i, j) as gridcells
	Inputs:
		points [list(tuple(i, j))]: List of grid coordinates
		z [float]: Height to print at
		pub [rospy.Publisher]: Publisher
		og [OccupancyGrid]: Reference grid
	Return: None
	"""
	
	# Creaty empty grid cells object
	grid_cells = GridCells()
	grid_cells.header.frame_id = og.header.frame_id
	grid_cells.cell_width = og.info.resolution
	grid_cells.cell_height = og.info.resolution

	# Convert points to grid cell array
	cells = []
	for point in points:
		i = point[0]
		j = point[1]
		x, y = og_ij_to_xy(i, j, og)
		cells.append(Point(x, y, z))
	grid_cells.cells = cells

	# Publish on given publisher
	pub.publish(grid_cells)
