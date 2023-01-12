import gym
import numpy as np

import model_predictive_speed_and_steer_control as mpc
from urdfenvs.robots.prius import Prius

DT = mpc.DT / 5


def run_prius(n_steps=10000, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel")
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=DT,
        robots=robots,
        render=render
    )

    pos0 = np.array([0.0, 0.0, 0.0])
    ob = env.reset(pos=pos0)

    history = []

    x_gym = np.zeros(4)

    x = y = yaw = v = 0
    a = d = yaw_prev = 0

    dl = 1.0  # course tick
    cx, cy, cyaw, ck = mpc.get_rrt_course(dl)
    sp = mpc.calc_speed_profile(cx, cy, cyaw, mpc.TARGET_SPEED)

    for i in range(n_steps):
        x_gym[0] = ob['robot_0']['joint_state']['position'][0]
        x_gym[1] = ob['robot_0']['joint_state']['position'][1]
        x_gym[2] += a * mpc.DT / DT * np.tan(d) / mpc.WB * mpc.DT
        x_gym[3] = ob['robot_0']['joint_state']['forward_velocity'][0]

        state = mpc.State(x, y, yaw, v)
        state = mpc.State(*x_gym)

        x, y, yaw, v, d, a = mpc.do_gym_simulation(
            cx, cy, cyaw, ck, sp, dl, state)

        print("->Gym state: ", x_gym,
              ob['robot_0']['joint_state']['steering'])
        print("->MPC state: ", [x, y, yaw, v, d])

        yr = np.clip((yaw - yaw_prev) / mpc.DT, -mpc.MAX_DSTEER, mpc.MAX_STEER)
        yaw_prev = yaw

        print("=>Actions: ", v, yr)

        for _ in range(int(mpc.DT / DT)):
            ob, _, _, _ = env.step([v, yr])

        history.append(ob)

    env.close()
    return history


if __name__ == "__main__":
    run_prius(render=True)
