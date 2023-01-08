import gym
import numpy as np
from MotionPlanningGoal.dynamicSubGoal import DynamicSubGoal

import MPC_luca
from urdfenvs.robots.prius import Prius


def run_prius(n_steps=500, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel")
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.03, robots=robots, render=render
    )

    pos0 = np.array([0.0, 0.0, 0.0])
    ob = env.reset(pos=pos0)

    history = []

    state = MPC_luca.State(*pos0, steering=0.0)

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

    actions = np.array([[0, 0]] * MPC_luca.T)
    xref = np.zeros((MPC_luca.NX, MPC_luca.T + 1))
    xref[:2, :] = 8.0

    dref = np.zeros((1, MPC_luca.T + 1))

    for i in range(n_steps):

        x0 = [state.x, state.y, state.yaw, state.steering]
        actions = np.array(MPC_luca.iterative_linear_mpc_control(
            xref, x0, dref, actions[:, 0], actions[:, 1])).T

        state = MPC_luca.update_state(state, *actions[0])

        # if ob['robot_0']['joint_state']['steering'] > MPC_luca.MAX_STEER \
        #         or ob['robot_0']['joint_state']['steering'] < -MPC_luca.MAX_STEER:
        #     action[1] = 0

        print(f"vel: {actions[0, 0]}   yaw_rate: {actions[0, 1]} \
              steering: {ob['robot_0']['joint_state']['steering']}")

        ob, _, _, _ = env.step(actions[0])

        history.append(ob)

    env.close()
    return history


if __name__ == "__main__":
    run_prius(render=True)
