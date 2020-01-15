import math

class car(object):
	"""docstring for car"""
	def __init__(self):
		self.v = 20
		self.ax = 10
		self.ay = 1
		self.az = 1
		self.maxt = 1/2

	def trayectory(self):
		x,y,z = [],[],[]
		track = []
		t = 0

		while t < self.maxt:
			x.append(self.v*t)
			y.append(self.ay*math.sin(((4*math.pi*self.v)/(self.ax))*t))
			z.append(self.ay*math.sin(((math.pi*self.v)/(self.ax))*t))
			t += 1/60

		track.append(x)
		track.append(y)
		track.append(z)

		return track

		