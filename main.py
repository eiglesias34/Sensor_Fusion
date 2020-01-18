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

    time = np.linspace(0, car.location[0] / car.v, 50)

    # TRAYECTORY

    trayectory = np.array([car.position(t) for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(trayectory[:, 0], trayectory[:, 1], trayectory[:, 2])
    plt.show()

    # Speed and Acceleration
    speeds = np.array([car.velocity(t) for t in time])
    accs = np.array([car.acceleration(t) for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(speeds[:, 0], speeds[:, 1], speeds[:, 2])
    plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(accs[:, 0], accs[:, 1], accs[:, 2])
    plt.show()

    # Velocity tangential vectors

    velocity_tan = np.array([car.velocity(t) / np.linalg.norm(car.velocity(t))
                         for t in time])


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trayectory[:, 0], trayectory[:, 1], trayectory[:, 2], color='blue')
    ax.quiver(trayectory[:, 0], trayectory[:, 1], trayectory[:, 2],
              velocity_tan[:, 0], velocity_tan[:, 1], velocity_tan[:, 2], color='orange')
    plt.show()

    # Modules part

    mod_vel = np.array([np.linalg.norm(sp) for sp in speeds])
    mod_accs = np.array([np.linalg.norm(acc) for acc in accs])

    dot_prodcuts = [np.dot(accs[i], speeds[i]) for i in range(len(accs))]

    plt.plot(time, mod_vel,
             time, mod_accs,
             time, dot_prodcuts)
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
    exercise_3()
    exercise_4()


if __name__ == "__main__":
    main()
