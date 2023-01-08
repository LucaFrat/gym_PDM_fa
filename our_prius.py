import math

import gym
import matplotlib.pyplot as plt
import numpy as np

import our_MPC
from urdfenvs.robots.prius import Prius

N_STEPS = 30
RENDER = False


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None


def run_prius(n_steps, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.2, robots=robots, render=render
    )

    # action = np.array([1, 0.4])
    pos0 = np.array([0.0, 0.0, 0.0])

    ob = env.reset(pos=pos0)
    print(f"Initial observation : {ob}")

    history = []
    action = np.array([0.0, 0.0])

    state = State(x=pos0[0], y=pos0[1], yaw=pos0[2], v=0.0)

    dl = 0.1
    cx, cy, cyaw, ck = our_MPC.get_forward_course(dl)
    sp = our_MPC.calc_speed_profile(cx, cy, cyaw, our_MPC.TARGET_SPEED)
    target_ind, _ = our_MPC.calc_nearest_index(state, cx, cy, cyaw, 0)
    cyaw = our_MPC.smooth_yaw(cyaw)
    oa, od = None, None

    # plot spline
    # for i, _ in enumerate(cx):
    #     print(cx[i], cy[i])
    #     plt.arrow(cx[i],
    #               cy[i],
    #               0.1 * math.cos(cyaw[i]),
    #               0.1 * math.sin(cyaw[i]),
    #               head_width=0.07)
    # plt.show()

    for i in range(n_steps):

        # xref, target_ind, dref = our_MPC.calc_ref_trajectory(
        #     state, cx, cy, cyaw, ck, sp, dl, target_ind)

        # for i, _ in enumerate(xref):
        #     print(cx[i], cy[i])
        #     plt.arrow(xref[i, 0],
        #               xref[i, 1],
        #               0.1 * math.cos(xref[i, 3]),
        #               0.1 * math.sin(xref[i, 3]),
        #               head_width=0.07)
        # plt.show()
        # print(xref)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        acc, delta = our_MPC.iterative_linear_mpc_control(
            xref, x0, dref, oa, od)

        if delta is not None:
            deltai, acci = delta[0], acc[0]

        state = our_MPC.update_state(state, acci, deltai)

        action[0] = acc[0]*our_MPC.DT + action[0]
        action[1] = action[0] * math.sin(delta[0]) / our_MPC.WB

        ob, _, _, _ = env.step(action)

        # print(f"{ob['robot_0']['joint_state']['position']}")

    history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_prius(n_steps=N_STEPS, render=RENDER)
