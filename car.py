import numpy as np


class Car:

	def __init__(self, v, a_x, a_y, a_z):
		self.v = v
		self.a_x = a_x
		self.a_y = a_y
		self.a_z = a_z

	def position(self, t):
		return np.array([
			self.v * t,
			self.a_y * np.sin((4 * np.pi * self.v / self.a_x) * t),
			self.a_z * np.sin((4 * np.pi * self.v / self.a_x) * t)
		])

	def velocity(self, t):
		return np.array([
			self.v,
			self.a_y * np.cos((4 * np.pi * self.v / self.a_x) * t) * (4 * np.pi * self.v / self.a_x),
			self.a_z * np.cos((4 * np.pi * self.v / self.a_x) * t) * (4 * np.pi * self.v / self.a_x)
		])

	def acceleration(self, t):
		return np.array([
			0,
			np.power(4 * np.pi * self.v / self.a_x, 2) * self.a_y * - np.sin((4 * np.pi * self.v / self.a_x) * t),
			np.power(4 * np.pi * self.v / self.a_x, 2) * self.a_z * - np.sin((4 * np.pi * self.v / self.a_x) * t)
		])

