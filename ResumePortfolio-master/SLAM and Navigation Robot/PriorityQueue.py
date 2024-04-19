"""
PriorityQueue.py
Class for implementing a priority queue
"""

import heapq

class PriorityQueue:

	def __init__(self):
		"""
		Brief: Constructs empty priority queue
		Inputs: None
		Return: None
		"""
		self.elements = []

	def empty(self):
		"""
		Brief: Returns true if queue is empty
		Inputs: None
		Return: Boolean
		"""
		return len(self.elements) == 0

	def put(self, item, priority):
		"""
		Brief: Adds item with given priority to queue
		Inputs:
			item [any type]: Item to add
			priority [number]: Low number = high priority
		Return: None
		"""
		heapq.heappush(self.elements, (priority, item))

	def pop(self):
		"""
		Brief: Removes and returns highest priority item
		Inputs: None
		Return: Item in queue
		"""
		return heapq.heappop(self.elements)[1]
