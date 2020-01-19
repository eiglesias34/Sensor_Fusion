import numpy as np


class Radar:

    def __init__(self, x, y, z, sigma_range, sigma_azimuth):

        self.location = np.array([
            x, y, z
        ])

        self.sigma_range = sigma_range
        self.sigma_azimuth = sigma_azimuth

    def measure(self, target):

        z_range = np.sqrt(
            np.power(target[0] - self.location[0], 2) +
            np.power(target[1] - self.location[1], 2) +
            np.power(target[2] - self.location[2], 2) -
            np.power(self.location[2], 2)
        )

        z_azimuth = np.arctan2(target[1] - self.location[1],
                               target[0] - self.location[0])

        return np.array([
            z_range,
            z_azimuth
        ]) + self.noise()

    def noise(self):

        return np.array([
            self.sigma_range * np.random.normal(0, 1, 1)[0],
            self.sigma_azimuth * np.random.normal(0, 1, 1)[0]
        ])


