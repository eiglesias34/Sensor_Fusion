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
			self.a_z * np.sin((np.pi * self.v / self.a_x) * t)
		])

	def velocity(self, t):
		return np.array([
			self.v,
			self.a_y * np.cos((4 * np.pi * self.v / self.a_x) * t) * (4 * np.pi * self.v / self.a_x),
			self.a_z * np.cos((np.pi * self.v / self.a_x) * t) * (np.pi * self.v / self.a_x)
		])

	def acceleration(self, t):
		return np.array([
			0,
			np.power(4 * np.pi * self.v / self.a_x, 2) * self.a_y * - np.sin((4 * np.pi * self.v / self.a_x) * t),
			np.power(np.pi * self.v / self.a_x, 2) * self.a_z * - np.sin((np.pi * self.v / self.a_x) * t)
		])

	def tangential(self,time):
		speed = np.array([self.velocity(t) for t in time])
		tangent = []
		for s in speed:
			module = np.power(np.power(s[0],2) + np.power(s[1],2) + np.power(s[2],2),1/2)
			tangent.append([s[0]/module,s[1]/module,s[2]/module])
		return np.array(tangent)

	def module(self,function):
		mod = []
		for f in function:
			mod.append(np.power((np.power(f[0],2) + np.power(f[1],2) + np.power(f[2],2)),1/2))
		return np.array(mod)

	def dot_product(self,f1,f2):
		result = []
		i = 0 
		while i < len(f1):
			result.append(f1[i,0]*f2[i,0]+f1[i,1]*f2[i,1]+f1[i,2]*f2[i,2])
			i += 1
		return np.array(result)


