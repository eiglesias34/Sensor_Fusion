import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from radar import Radar
from kalman import KalmanFilter
from parameters import target, radars


def exercise_3():

    car = target

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
    velocity_tan = np.array([car.velocity(t) / np.linalg.norm(car.velocity(t))
                            for t in time])

    # The 500 factor is just a scale
    # so that the arrows can be seen in the plot
    scaled_velocity_tan = 500 * velocity_tan

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
            color='r', label='Car trajectory')
    ax.quiver(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
              scaled_velocity_tan[:, 0], scaled_velocity_tan[:, 1], scaled_velocity_tan[:, 2],
              color='b', label='Tangential Velocity (SCALED * 500)')
    plt.title('Car Tangential Vectors')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    plt.legend()
    plt.show()

    # 3.4 Modules part

    mod_vel = np.array([np.linalg.norm(sp) for sp in velocities])
    mod_accs = np.array([np.linalg.norm(acc) for acc in accs])

    acc_tans = [np.matmul(accs[i], velocity_tan[i]) for i in range(len(accs))]


    fig = plt.figure()
    plt.plot(time, mod_vel)
    plt.title('Evolution of the Velocity Vector Norm in Time')
    plt.xlabel('Velocity Vector Norm')
    plt.ylabel('Time (seconds)')
    plt.legend()

    plt.show()

    fig = plt.figure()
    plt.plot(time, mod_accs)
    plt.title('Evolution of the Acceleration Vector Norm in Time')
    plt.xlabel('Acceleration Vector Norm')
    plt.ylabel('Time (seconds)')
    plt.show()

    fig = plt.figure()
    plt.plot(time, acc_tans)
    plt.title('Evolution of Acceleration Vector x Tangential Vectors in Time')
    plt.xlabel('r\'\'(t)t(t)')
    plt.ylabel('Time (seconds)')
    plt.show()

def exercise_4():

    colors = ['b', 'g', 'm', 'c', 'y']
    car = target

    time = np.linspace(0, car.location[0] / car.v, 900)

    trajectory = np.array([car.position(t) for t in time])

    # 4.2 Measurements transformation

    fig = plt.figure()

    for i, radar in enumerate(radars, start=1):
        measurements = [radar.measure(car, t) for t in time]
        trans_measures = np.array([Radar.cartesian_measurement(m)
                                   + radar.location[:2]
                                   for m in measurements])
        plt.plot(trans_measures[:, 0], trans_measures[:, 1],
                 c=colors[(i % 5) - 1], label='Radar %s Measurements' % i)

    plt.plot(trajectory[:, 0], trajectory[:, 1], c='r', label='Real trajectory')
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
    # plt.show()

    kalman.d_retro(time_limit)
    #
    fig = plt.figure()
    plt.plot(trajectory[:, 0], trajectory[:, 1], c='r', label='Target Trajectory')
    plt.plot(kalman.no_filtered_track[:, 0], kalman.no_filtered_track[:, 1],
             c='y', label='Predicted Track')
    plt.plot(kalman.track[:, 0], kalman.track[:, 1],
             c='g', label='Filtered Track')
    plt.plot(kalman.retro_track[:, 0], kalman.retro_track[:, 1],
             c='b', label='Track with retrodiction')
    plt.xlabel('X-axis (m)')
    plt.ylabel('Y-axis (m)')
    plt.title('Real Trajectory vs Track using Kalman Filter vs Track using retrodiction')
    plt.legend(loc='upper right')
    plt.xlim(-1000, 12000)
    plt.ylim(-2000, 2000)
    plt.show()

    # error calculation:
    err = np.sum(np.sqrt(np.sum(
        (trajectory[:, 0] - kalman.retro_track[:, 0]) ** 2 +
        (trajectory[:, 1] - kalman.retro_track[:, 1]) ** 2
    )))
    print('Error of the track when applying retrodiction: ')
    print(err)

    # error calculation:
    err = np.sum(np.sqrt(np.sum(
        (trajectory[:, 0] - kalman.track[:, 0]) ** 2 +
        (trajectory[:, 1] - kalman.track[:, 1]) ** 2
    )))
    print('Error of the track without retrodiction: ')
    print(err)


def main():

    exercise_3()
    exercise_4()


if __name__ == "__main__":
    main()
