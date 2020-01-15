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

    time = np.linspace(0, a_x / v, 50)

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


def main():
    exercise_3()


if __name__ == "__main__":
    main()
