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

    # 3.1 trajectory

    trajectory = np.array([car.position(t) for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label='Car trajectory')
    plt.title('Car Trajectory')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    plt.legend()
    plt.show()

    # 3.2 Velocity and Acceleration
    velocities = np.array([car.velocity(t) for t in time])
    accs = np.array([car.acceleration(t) for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(velocities[:, 0], velocities[:, 1], velocities[:, 2], label='Velocity')
    plt.title('Car Velocity m/s')
    ax.set_xlabel('X-axis (m)')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(accs[:, 0], accs[:, 1], accs[:, 2], label='Acceleration')
    plt.title('Car Acceleration m/s^{2}')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    plt.show()

    # 3.3 Velocity tangential vectors
    # The 500 factor is just a scale
    # so that the arrows can be seen in the plot
    velocity_tan = np.array([500 * car.velocity(t) / np.linalg.norm(car.velocity(t))
                            for t in time])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
            color='r', label='Car trajectory')
    ax.quiver(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
              velocity_tan[:, 0], velocity_tan[:, 1], velocity_tan[:, 2],
              color='b', label='Tangential Velocity (SCALED * 500)')
    plt.title('Car Tangential Vectors')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    plt.show()

    # 3.4 Modules part

    mod_vel = np.array([np.linalg.norm(sp) for sp in velocities])
    mod_accs = np.array([np.linalg.norm(acc) for acc in accs])

    dot_products = [np.dot(accs[i], velocities[i]) for i in range(len(accs))]

    fig = plt.figure()
    plt.plot(time, mod_vel,
             time, mod_accs,
             time, dot_products)
    plt.show()


def exercise_4():

    colors = ['b', 'g', 'm', 'c', 'y']
    radars = [
        Radar(x=0.0, y=100000.0, z=10000.0, sigma_range=10.0, sigma_azimuth=0.5),
        Radar(x=100000.0, y=0.0, z=10000.0, sigma_range=20.0, sigma_azimuth=0.5),
        # Radar(x=100000.0, y=100000.0, z=10000.0, sigma_range=10.0, sigma_azimuth=0.1),
        # Radar(x=0.0, y=-100000.0, z=10000.0, sigma_range=10.0, sigma_azimuth=0.1),
    ]

    car = Car(v=20 * 1000 / 3600,
              a_x=10000.0,
              a_y=1000.0,
              a_z=1000.0)

    time = np.linspace(0, car.location[0] / car.v, 900)

    trajectory = np.array([car.position(t) for t in time])

    # 4.2 Measurements transformation

    fig = plt.figure()

    for i, radar in enumerate(radars, start=1):
        trans_measures = np.array([radar.cartesian_measure(car, t) for t in time])
        plt.plot(trans_measures[:, 0], trans_measures[:, 1],
                 c=colors[(i % 5) - 1], label='Radar %s Measurements' % i)

    plt.plot(trajectory[:, 0], trajectory[:, 1], c='r')
    plt.title('Trajectory and Radars Measurements')
    plt.legend()
    plt.xlabel('X-axis (m)')
    plt.ylabel('Y-axis (m)')
    plt.show()

    # 4.3 Kalman Filter

    time_limit = car.location[0] / car.v
    kalman = KalmanFilter(radars, car, delta_t=2)
    kalman.filter(time_limit)

    fig = plt.figure()
    plt.plot(trajectory[:, 0], trajectory[:, 1], c='r', label='Target Trajectory')
    plt.plot(kalman.track[:, 0], kalman.track[:, 1],
             c='g', label='Track')
    plt.xlabel('X-axis (m)')
    plt.ylabel('Y-axis (m)')
    plt.title('Real Trajectory vs Track using Kalman Filter')
    plt.legend(loc='upper right')
    plt.xlim(-1000, 12000)
    plt.ylim(-2000, 2000)
    plt.show()

    kalman.d_retro(time_limit)
    #
    fig = plt.figure()
    plt.plot(trajectory[:, 0], trajectory[:, 1], c='r', label='Target Trajectory')
    plt.plot(kalman.track[:, 0], kalman.track[:, 1],
             c='g', label='Track')
    plt.xlabel('X-axis (m)')
    plt.ylabel('Y-axis (m)')
    plt.title('Real Trajectory vs Track using Kalman Filter')
    plt.legend(loc='upper right')
    # plt.xlim(-1000, 12000)
    # plt.ylim(-2000, 2000)
    plt.show()


def main():

    # exercise_3()
    exercise_4()


if __name__ == "__main__":
    main()
