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
        self.no_filtered_track = np.array([])
        self.no_filtered_Ps = np.array([])
        self.retro_track = np.zeros((900, 6))
        self.retro_Ps = np.zeros((900, 6, 6))

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

        D = (sigma ** 2) * D

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

        P = np.identity(6)
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

        t = 0

        while t < time_limit:

            # obtain effective measurement and cov
            zk, Rk = self.get_effective_measurement(t)

            target_state, P = self.prediction_step(target_state, P)
            target_state, P = self.correction_step(zk, Rk, target_state, P)
            self.track = np.array([target_state]) if t == 0 else np.vstack((self.track, target_state))
            self.Ps = np.array([P.copy()]) if t == 0 else np.vstack([self.Ps, [P]])

            t += self.delta_t

    def d_retro(self, time_limit):

        P = np.identity(6)
        F = np.identity(6)
        F[:3, 3:] = self.delta_t * np.identity(3)

        # Initial target state
        target_state = np.array([
            self.target.position(0)[0],
            self.target.position(0)[1],
            self.target.position(0)[2],
            self.target.velocity(0)[0],
            self.target.velocity(0)[1],
            self.target.velocity(0)[2]
        ])

        t = 0

        while t < time_limit:

            # obtain effective measurement and cov
            zk, Rk = self.get_effective_measurement(t)

            target_state_nf, P_nf = self.prediction_step(target_state, P)

            # Save no filtered track and Ps
            self.no_filtered_track = np.array([target_state_nf.copy()]) if t == 0 \
                else np.vstack((self.no_filtered_track, target_state_nf.copy()))
            self.no_filtered_Ps = np.array([P_nf.copy()]) if t == 0 \
                else np.vstack([self.no_filtered_Ps, [P_nf.copy()]])

            target_state, P = self.correction_step(zk, Rk, target_state_nf, P_nf)

            # Save filtered track and Ps
            self.track = np.array([target_state.copy()]) if t == 0 else np.vstack((self.track, target_state.copy()))
            self.Ps = np.array([P.copy()]) if t == 0 else np.vstack([self.Ps, [P.copy()]])

            # retrodiction:
            i = int(t / 2)
            self.retro_track[i] = self.track[i].copy()
            self.retro_Ps[i] = self.Ps[i].copy()

            for l in range(len(self.track) - 2, -1, -1):
                W = np.matmul(np.matmul(self.Ps[l], F.T), np.linalg.inv(self.no_filtered_Ps[l + 1]))
                self.retro_track[l] = self.track[l] \
                                      + np.matmul(W, self.retro_track[l + 1] - self.no_filtered_track[l + 1])

                self.retro_Ps[l] = self.Ps[l] \
                                   + np.matmul(np.matmul(W, self.retro_Ps[l + 1] - self.no_filtered_Ps[l + 1]), W.T)

            t += self.delta_t

    def get_effective_measurement(self, t):

        # individual measures in array
        zs = np.array([
            sensor.measure(self.target, t)
            for sensor in self.radars
        ])

        # convert to cartesian
        cart_zs = np.array([
            Radar.cartesian_measurement(measurement)
            for measurement in zs
        ])

        # add the radar's position
        cart_zs = np.array([
           cart_zs[i] + self.radars[i].location[:2]
           for i in range(len(self.radars))
        ])

        # inverted individual covs (R_k^{-1})
        Rs = [
            np.linalg.inv(
                Radar.cartesian_error_covariance(
                    zs[i],
                    self.radars[i].sigma_range,
                    self.radars[i].sigma_azimuth))
            for i in range(len(self.radars))]

        # effective cov
        Rk = np.linalg.inv(np.sum(Rs, axis=0))

        # values for the sum to obtain z_k
        zk_sum_values = np.array([
            np.matmul(Rs[i], cart_zs[i])
            for i in range(len(self.radars))
        ])

        # effective measurement
        zk = np.matmul(Rk, np.sum(zk_sum_values, axis=0))

        return zk, Rk
