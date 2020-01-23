from car import Car
from radar import Radar

target = Car(v=20 * 1000 / 3600,
             a_x=10000.0,
             a_y=1000.0,
             a_z=1000.0)

radars = [
        Radar(x=0.0, y=100000.0, z=10000.0, sigma_range=10.0, sigma_azimuth=0.1),
        Radar(x=100000.0, y=0.0, z=10000.0, sigma_range=10.0, sigma_azimuth=0.1),
        # Radar(x=100000.0, y=100000.0, z=10000.0, sigma_range=10.0, sigma_azimuth=0.1),
        # Radar(x=0.0, y=-100000.0, z=10000.0, sigma_range=10.0, sigma_azimuth=0.1),
    ]
