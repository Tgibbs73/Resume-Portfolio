"""
Queue.py
Class for implementing a queue
"""

class Queue:
	def __init__(self):
		"""
		Brief: Constructs empty queue
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

	def put(self, item):
		"""
		Brief: Adds item to queue
		Inputs: item [any type]
		Return: None
		"""
		self.elements.insert(0, item)

	def pop(self):
		"""
		Brief: Removes and returns item at front of queue
		Inputs: None
		Return: item [any type]
		"""
		return self.elements.pop()
