import numpy as np


class Car:

	def __init__(self, v, a_x, a_y, a_z):
		self.location = np.array([
			a_x,
			a_y,
			a_z
		])
		self.v = v

	def position(self, t):
		return np.array([
			self.v * t,
			self.location[1] * np.sin((4 * np.pi * self.v / self.location[0]) * t),
			self.location[2] * np.sin((np.pi * self.v / self.location[0]) * t)
		])

	def velocity(self, t):
		return np.array([
			self.v,
			self.location[1] * np.cos((4 * np.pi * self.v / self.location[0]) * t)
				* (4 * np.pi * self.v / self.location[0]),
			self.location[2] * np.cos((np.pi * self.v / self.location[0]) * t)
				* (np.pi * self.v / self.location[0])
		])

	def acceleration(self, t):
		return np.array([
			0,
			np.power(4 * np.pi * self.v / self.location[0], 2)
				* self.a_y * - np.sin((4 * np.pi * self.v / self.location[0]) * t),
			np.power(np.pi * self.v / self.location[0], 2)
				* self.a_z * - np.sin((np.pi * self.v / self.location[0]) * t)
		])

