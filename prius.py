import gym
import numpy as np
from MotionPlanningGoal.dynamicSubGoal import DynamicSubGoal

import mpc
from urdfenvs.robots.prius import Prius

DT = 0.01


def run_prius(n_steps=10000, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel")
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=mpc.DT,
        robots=robots,
        render=render
    )

    pos0 = np.array([0.0, 0.0, 0.0])
    ob = env.reset(pos=pos0)

    history = []

    splineGoalDict = {
        "weight": 1.0,
        "is_primary_goal": True,
        'indices': [0, 1, 2],
        'parent_link': 0,
        'child_link': 3,
        'trajectory': {
            'degree': 2,
            'controlPoints': [
                [8.0, 8.0, 0.0],
                [8.0, 8.0, 0.0],
                [8.0, 8.0, 0.0]],
            'duration': 10},
        'epsilon': 0.08,
        'type': "splineSubGoal",
    }
    splineGoal = DynamicSubGoal(name="goal3", content_dict=splineGoalDict)
    env.add_goal(splineGoal)

    x = np.zeros(4)

    actions = np.array([[0, 0]] * mpc.T)

    # goal state
    xref = np.array([8.0, 8.0, 1.0, -0.5])

    for i in range(n_steps):
        x[0] = ob['robot_0']['joint_state']['position'][0]
        x[1] = ob['robot_0']['joint_state']['position'][1]
        x[2] += actions[0, 0] * mpc.DT / DT * \
            np.tan(x[3]) / mpc.WB * mpc.DT
        x[3] = ob['robot_0']['joint_state']['steering']
        print("state: ", x)

        actions = np.array(mpc.iterative_linear_mpc_control(
            x,
            xref,
            actions[:, 0],
            actions[:, 1])
        ).T

        print("actions: ", actions[0, :])

        for _ in range(int(mpc.DT / DT)):
            ob, _, _, _ = env.step(actions[0])

        history.append(ob)

    env.close()
    return history


if __name__ == "__main__":
    run_prius(render=True)
