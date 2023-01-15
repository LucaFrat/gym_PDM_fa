import math

import matplotlib.pyplot as plt
import numpy as np

import utils.mpc_utils as mpc
import utils.plot_utils as plot
import utils.specs_utils as specs
import utils.trajectory_utils as trajectory
from environ_obstacles import environ


def check_goal(state, goal, tind, nind):
    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= specs.GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= specs.STOP_SPEED)

    if isgoal and isstop:
        return True

    return False


def do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state):
    """
    Simulation
    cx: course x position list
    cy: course y position list
    cy: course yaw position list
    ck: course curvature list
    sp: speed profile
    dl: course tick [m]
    """

    goal = [cx[-1], cy[-1]]

    state = initial_state

    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]
    target_ind, _ = trajectory.calc_nearest_index(state, cx, cy, cyaw, 0)

    odelta, oa = None, None

    cyaw = trajectory.smooth_yaw(cyaw)

    obstacleList = environ()
    x_inc, y_inc = 0.0, 0.0

    while specs.MAX_TIME >= time:
        xref, target_ind, dref = trajectory.calc_ref_trajectory(
            state, cx, cy, cyaw, ck, sp, dl, target_ind)

        x0 = [state.x, state.y, state.v, state.yaw]

        origin_obst = np.array([[8+2*x_inc, 20-y_inc], [-10+x_inc, -20+1.8*y_inc],
                               [-20-1.1*x_inc, 20-0.9*y_inc], [12+x_inc, -30+1.5*y_inc]])
        x_inc -= 0.1
        y_inc += 0.2

        oa, odelta, ox, oy, _, _ = mpc.iterative_linear_mpc_control(
            xref, x0, dref, oa, odelta, origin_obst)

        if odelta is not None:
            di, ai = odelta[0], oa[0]

        state = mpc.update_state(state, ai, di)
        time = time + specs.DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(di)
        a.append(ai)

        if check_goal(state, goal, target_ind, len(cx)):
            print("Goal")
            break

        if specs.show_animation:

            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            if ox is not None:
                plt.plot(ox, oy, "xr", label="MPC")

            for obst in obstacleList:
                plot.plot_env(*obst)

            for j in range(origin_obst.shape[0]):
                plt.plot(origin_obst[j][0],
                         origin_obst[j][1],
                         marker='o',
                         linewidth=7)

            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plot.plot_car(state.x, state.y, state.yaw, steer=di)
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(time, 2))
                      + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
            plt.pause(0.0001)
            plt.cla()

    return t, x, y, yaw, v, d, a


def main():
    print(__file__ + " start!!")

    dl = 1.0  # course tick

    # cx, cy, cyaw, ck = get_switch_back_course(dl)
    cx, cy, cyaw, ck = trajectory.get_rrt_course(dl)

    sp = trajectory.calc_speed_profile(cx, cy, cyaw, specs.TARGET_SPEED)

    initial_state = mpc.State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

    t, x, y, yaw, v, d, a = do_simulation(
        cx, cy, cyaw, ck, sp, dl, initial_state)

    if specs.show_animation:
        plt.close("all")
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

        plt.show()

    return a, d


if __name__ == '__main__':
    main()
