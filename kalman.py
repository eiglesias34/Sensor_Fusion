import matplotlib.pyplot as plt
import numpy as np

from car import Car
from radar import Radar


class KalmanFilter:

    def __init__(self, radars: [Radar], target: Car, P, delta_t):
        self.radars = radars
        self.target = target
        self.P = P
        self.delta_t = delta_t
        self.H = np.array([
                            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
                            ])

    def prediction_step(self, x, P):

        # TODO: check if this is a valid sigma
        sigma = 1.0

        # Construct F
        F = np.identity(6)
        F[:3, 3:] = self.delta_t * np.identity(1)

        # Construct D
        D = np.zeros((6, 6))
        D[:3, :3] = 1.0 / 4 * (self.delta_t ** 4) * np.identity(3)
        D[:3, 3:] = 1.0 / 2 * (self.delta_t ** 3) * np.identity(3)
        D[3:, :3] = 1.0 / 2 * (self.delta_t ** 3) * np.identity(3)
        D[3:, 3:] = sigma * (self.delta_t ** 3) * np.identity(3)

        # Predict
        x1 = np.matmul(F, x)
        P1 = np.matmul(np.matmul(F, P), F.T) + D

        return x1, P1

    def correction_step(self, z, x, P):

        # Construct R
        # R = ?

        v = z - np.matmul(self.H, x)
        S = np.matmul(np.matmul(self.H, P), self.H.T) + R
        W = np.matmul(np.matmul(P, self.H.T), np.invert(S))

        x += np.matmul(W, v)
        P -= np.matmul(np.matmul(W, S), W.T)

        return x, P

    def filter(self):
        time_limit = self.target.location[0] / self.target.v

        t = 0

        radar = self.radars[0]

        while t <= time_limit:

            target_state = np.array([
                                self.target.position(t)[0],
                                self.target.position(t)[1],
                                self.target.position(t)[2],
                                self.target.velocity(t)[0],
                                self.target.velocity(t)[1],
                                self.target.velocity(t)[2]
                                ])

            z = radar.measure(self.target.position(t))

            x, self.P = self.prediction_step(target_state, self.P)
            x, self.P = self.correction_step(z, x, self.P)

            t += self.delta_t
