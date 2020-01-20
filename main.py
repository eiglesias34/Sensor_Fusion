import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from car import Car
from radar import Radar
from kalman import KalmanFilter


def exercise_3():

    car = Car(v=20 * 1000 / 3600,
              a_x=10000.0,
              a_y=1000.0,
              a_z=1000.0)

    time = np.linspace(0, car.location[0] / car.v, 50)

    # trajectory

    trajectory = np.array([car.position(t) for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2])
    plt.title('Car Trajectory')
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    plt.show()

    # Speed and Acceleration
    speeds = np.array([car.velocity(t) for t in time])
    accs = np.array([car.acceleration(t) for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(speeds[:, 0], speeds[:, 1], speeds[:, 2])
    plt.title('Car Velocity m/s')
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(accs[:, 0], accs[:, 1], accs[:, 2])
    plt.title('Car Acceleration m/s^{2}')
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    plt.show()

    # Velocity tangential vectors
    # The 500 factor is just a scale
    # so that the arrows can be seen in the plot
    velocity_tan = np.array([500 * car.velocity(t) / np.linalg.norm(car.velocity(t))
                            for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color='blue')
    ax.quiver(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
              velocity_tan[:, 0], velocity_tan[:, 1], velocity_tan[:, 2], color='orange')
    plt.title('Car Tangential Vectors')
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    plt.show()

    # Modules part

    mod_vel = np.array([np.linalg.norm(sp) for sp in speeds])
    mod_accs = np.array([np.linalg.norm(acc) for acc in accs])

    dot_products = [np.dot(accs[i], speeds[i]) for i in range(len(accs))]

    fig = plt.figure()
    plt.plot(time, mod_vel,
             time, mod_accs,
             time, dot_products)
    plt.show()


def exercise_4():

    radar_1 = Radar(x=0.0, y=100000.0, z=10000.0, sigma_range=10.0, sigma_azimuth=0.1)
    radar_2 = Radar(x=100000.0, y=0.0, z=10000.0, sigma_range=10.0, sigma_azimuth=0.1)

    car = Car(v=20 * 1000 / 3600,
              a_x=10000.0,
              a_y=1000.0,
              a_z=1000.0)

    time = np.linspace(0, car.location[0] / car.v, 900)

    trajectory = np.array([car.position(t) for t in time])

    # Measurements transformation
    trans_measures_1 = np.array([radar_1.cartesian_measure(car, t) for t in time])
    trans_measures_2 = np.array([radar_2.cartesian_measure(car, t) for t in time])
    fig = plt.figure()
    plt.plot(trans_measures_1[:, 0], trans_measures_1[:, 1], c='b')
    plt.plot(trans_measures_2[:, 0], trans_measures_2[:, 1], c='g')
    plt.plot(trajectory[:, 0], trajectory[:, 1], c='r')
    plt.show()

    # High Variance
    P = np.identity(6)

    kalman = KalmanFilter([radar_1, radar_2], car, P, 2)

    kalman.filter()


def main():
    # exercise_3()
    exercise_4()


if __name__ == "__main__":
    main()
