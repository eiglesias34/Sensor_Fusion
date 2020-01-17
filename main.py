import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from car import Car
from radar import Radar

def car_pos(t, v, a_x, a_y, a_z):

    return np.array([
        v*t,
        a_y * np.sin((4 * np.pi * v / a_x) * t),
        a_z * np.sin((4 * np.pi * v / a_x) * t)
    ])


def exercise_3():

    car = Car(v=20, a_x=10, a_y=1, a_z=1)

    time = np.linspace(0, car.a_x / car.v, 50)

    trayectory = np.array([car.position(t) for t in time])
    velocity = np.array([car.velocity(t) / np.linalg.norm(car.velocity(t))
                         for t in time])

    acceleration = np.array([car.acceleration(t) / np.linalg.norm(car.acceleration(t))
                          for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(trayectory[:, 0], trayectory[:, 1], trayectory[:, 2])
    # plt.show()

    ax.quiver(trayectory[:, 0], trayectory[:, 1], trayectory[:, 2],
              velocity[:, 0], velocity[:, 1], velocity[:, 2], color='orange')
    ax.quiver(trayectory[:, 0], trayectory[:, 1], trayectory[:, 2],
              acceleration[:, 0], acceleration[:, 1], acceleration[:, 2], color='blue')


    plt.show()

def exercise_4():

    radar_1 = Radar(x=0.0, y=100.0, z=10.0, sigma_range=10.0, sigma_azimuth=0.1)
    radar_2 = Radar(x=100.0, y=0.0, z=10.0, sigma_range=10.0, sigma_azimuth=0.1)

    car = Car(v=20.0, a_x=10.0, a_y=1.0, a_z=1.0)

    time = np.linspace(0, car.location[0] / car.v, 50)

    measurements_1 = np.zeros((len(time), 2))
    measurements_2 = np.zeros((len(time), 2))

    measurements_1 = [radar_1.measure(car.position(t)) for t in time]
    measurements_2 = [radar_2.measure(car.position(t)) for t in time]

    print('ola')
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')


    #
    # ax.scatter([radar_1.location[0]], [radar_1.location[1]], [radar_1.location[2]], c='b')
    # ax.scatter([radar_2.location[0]], [radar_2.location[1]], [radar_2.location[2]], c='r')
    # plt.show()
    #

def main():
    # exercise_3()
    exercise_4()


if __name__ == "__main__":
    main()
