import matplotlib.pyplot as plt
import numpy as np

from car import Car
from radar import Radar


class KalmanFilter:

    def __init__(self, radars: [Radar], target: Car, delta_t):
        self.radars = radars
        self.target = target
        self.delta_t = delta_t
        self.H = np.array([
                            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
                            ])
        self.track = np.array([])
        self.Ps = np.array([])

    def prediction_step(self, x, P):

        # TODO: check if this is a valid sigma
        sigma = 1.0

        # Construct F
        F = np.identity(6)
        F[:3, 3:] = self.delta_t * np.identity(3)

        # Construct D
        D = np.zeros((6, 6))
        D[:3, :3] = 1.0 / 4 * (self.delta_t ** 4) * np.identity(3)
        D[:3, 3:] = 1.0 / 2 * (self.delta_t ** 3) * np.identity(3)
        D[3:, :3] = 1.0 / 2 * (self.delta_t ** 3) * np.identity(3)
        D[3:, 3:] = (self.delta_t ** 2) * np.identity(3)

        D = sigma * D

        # Predict
        x1 = np.matmul(F, x)
        P1 = np.matmul(np.matmul(F, P), F.T) + D

        return x1, P1

    def correction_step(self, z, R, x, P):

        np.matmul(self.H, x)
        v = z - np.matmul(self.H, x)
        S = np.matmul(np.matmul(self.H, P), self.H.T) + R
        W = np.matmul(np.matmul(P, self.H.T), np.linalg.inv(S))

        x += np.matmul(W, v)
        P -= np.matmul(np.matmul(W, S), W.T)

        return x, P

    def filter(self, time_limit):

        t = 0

        P = np.identity(6)
        time = np.linspace(0, self.target.location[0] / self.target.v, 900)

        x = np.zeros(6)

        # Initial target state
        target_state = np.array([
            self.target.position(0)[0],
            self.target.position(0)[1],
            self.target.position(0)[2],
            self.target.velocity(0)[0],
            self.target.velocity(0)[1],
            self.target.velocity(0)[2]
        ])

        while t <= time_limit:

            # individual measures in array
            z = np.array([
                sensor.cartesian_measure(self.target, t)
                for sensor in self.radars
            ])

            # inverted individual covs (R_k^{-1})
            Rs = [np.linalg.inv(sensor.cartesian_error_covariance(self.target, t))
                  for sensor in self.radars]

            # effective cov
            Rk = np.linalg.inv(np.sum(Rs, axis=0))

            # values for the sum to obtain z_k
            zk_sum_values = np.array([
                np.matmul(Rs[i], z[i])
                for i in range(len(self.radars))
            ])

            # effective measurement
            zk = np.matmul(Rk, np.sum(zk_sum_values, axis=0))

            target_state, P = self.prediction_step(target_state, P)
            target_state, P = self.correction_step(zk, Rk, x, P)
            self.track = np.array([target_state]) if t == 0 else np.vstack((self.track, target_state))
            self.Ps = np.array([P.copy()]) if t == 0 else np.vstack([self.Ps, [P]])

            t += self.delta_t

    def d_retro(self, time_limit):

        t = 0

        P = np.identity(6)
        F = np.identity(6)
        F[:3, 3:] = self.delta_t * np.identity(3)

        x = np.zeros(6)

        # Initial target state
        target_state = np.array([
            self.target.position(0)[0],
            self.target.position(0)[1],
            self.target.position(0)[2],
            self.target.velocity(0)[0],
            self.target.velocity(0)[1],
            self.target.velocity(0)[2]
        ])

        while t <= time_limit:
            # individual measures in array
            z = np.array([
                sensor.cartesian_measure(self.target, t)
                for sensor in self.radars
            ])

            # inverted individual covs (R_k^{-1})
            Rs = [np.linalg.inv(sensor.cartesian_error_covariance(self.target, t))
                  for sensor in self.radars]

            # effective cov
            Rk = np.linalg.inv(np.sum(Rs, axis=0))

            # values for the sum to obtain z_k
            zk_sum_values = np.array([
                np.matmul(Rs[i], z[i])
                for i in range(len(self.radars))
            ])

            # effective measurement
            zk = np.matmul(Rk, np.sum(zk_sum_values, axis=0))

            target_state, P = self.prediction_step(target_state, P)
            target_state, P = self.correction_step(zk, Rk, x, P)

            self.track = np.array([target_state]) if t == 0 else np.vstack((self.track, target_state))
            self.Ps = np.array([P.copy()]) if t == 0 else np.vstack([self.Ps, [P]])

            # retrodiction:
            # for l in range(t - self.delta_t, 0, -self.delta_t):
            for l in range(len(self.track) - 1, 0, -1):
                W = np.matmul(np.matmul(self.Ps[l - 1], F), np.linalg.inv(self.Ps[l]))
                self.track[l - 1] = self.track[l - 1] \
                                    + np.matmul(W, self.track[l - 1] - self.track[l])
            t += self.delta_t

    #   Noisy approach (?)
    # def discrete_retrodiction(self):
    #
    #     track = np.array(self.track)
    #     Ps = np.array(self.Ps)
    #
    #     F = np.identity(6)
    #     F[:3, 3:] = self.delta_t * np.identity(3)
    #
    #     for i in range(len(track))[::-1]:
    #         if i == 0:
    #             break
    #         W = np.matmul(np.matmul(Ps[i - 1], F), np.linalg.inv(Ps[i]))
    #
    #         track[i] = track[i] + np.matmul(W, track[i] - track[i-1])
    #
    #     return track
