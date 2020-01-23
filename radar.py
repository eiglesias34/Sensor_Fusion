import numpy as np


class Radar:

    def __init__(self, x, y, z, sigma_range, sigma_azimuth):

        self.location = np.array([
            x, y, z
        ])

        self.sigma_range = sigma_range
        self.sigma_azimuth = sigma_azimuth * np.pi / 180

    def noise(self):

        return np.array([
            self.sigma_range * np.random.normal(0, 1, 1)[0],
            self.sigma_azimuth * np.random.normal(0, 1, 1)[0]
        ])

    def measure(self, target, t):

        target_position = target.position(t)
        z_range = np.sqrt(
            (target_position[0] - self.location[0]) ** 2 +
            (target_position[1] - self.location[1]) ** 2 +
            (target_position[2] - self.location[2]) ** 2 -
            self.location[2] ** 2
        )

        z_azimuth = np.arctan2(target_position[1] - self.location[1],
                               target_position[0] - self.location[0])

        return np.array([
            z_range,
            z_azimuth
        ]) + self.noise()

    @staticmethod
    def cartesian_measurement(z):

        return z[0] * np.array([np.cos(z[1]), np.sin(z[1])])

    @staticmethod
    def cartesian_error_covariance(z, sigma_range, sigma_azimuth):

        r, phi = z[0], z[1]
        D = np.array([
            [np.cos(phi), - np.sin(phi)],
            [np.sin(phi), np.cos(phi)]
        ])

        core_matrix = np.array([
            [sigma_range ** 2, 0],
            [0, (r * sigma_azimuth) ** 2]
        ])

        trt = np.matmul(np.matmul(D, core_matrix), D.T)

        return trt



