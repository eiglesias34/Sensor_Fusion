import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from car import Car


def car_pos(t, v, a_x, a_y, a_z):

    return np.array([
        v*t,
        a_y * np.sin((4 * np.pi * v / a_x) * t),
        a_z * np.sin((4 * np.pi * v / a_x) * t)
    ])


def exercise_3():

    v = 20
    a_x = 10
    a_y = a_z = 1

    car = Car(v=20, a_x=10, a_y=1, a_z=1)

    time = np.linspace(0, a_x / v, 200)

    trayectory = np.array([car.position(t) for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(trayectory[:, 0], trayectory[:, 1], trayectory[:, 2])
    plt.show()

    speed = np.array([car.velocity(t) for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(speed[:, 0], speed[:, 1], speed[:, 2])
    plt.show()

    acc = np.array([car.acceleration(t) for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(acc[:, 0], acc[:, 1], acc[:, 2])
    plt.show()

    tangent = car.tangential(time)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(tangent[:, 0], tangent[:, 1], tangent[:, 2])
    plt.show()

    mod_vel = car.module(speed)
    mod_acc = car.module(acc)
    dot = car.dot_product(acc,tangent)

    plt.plot(time,mod_vel,
            time,mod_acc,
            time,dot)
    plt.show()


def main():
    exercise_3()


if __name__ == "__main__":
    main()
