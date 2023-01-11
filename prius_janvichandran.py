import gym
from urdfenvs.robots.prius import Prius
import numpy as np


def run_prius(n_steps=2000, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    action = np.array([1.1, 0.1])
    pos0 = np.array([-8, 10, -1.0])
    ob = env.reset(pos=pos0)
    print(f"Initial observation : {ob}")
    if obstacles:
        from examples.scene_objects.obstacles import (
            urdfObst2,
            urdfObst3,
            urdfObst4,
            urdfObst5,
            urdfObst6,
            urdfObst7,
            urdfObst8,
            urdfObst9,
            urdfObst10,
            urdfObst11,
            urdfObst12,
            urdfObst13,
            urdfObst14,
            urdfObst15,
            urdfObst16,
            urdfObst17,
            urdfObst18,
            urdfObst19,
            urdfObst20,
            urdfObst21,
            urdfObst22,
            urdfObst23,
            urdfObst24,
            urdfObst25,
            urdfObst26,
            urdfObst27,
            urdfObst28,
            urdfObst29,
            urdfObst30,
            urdfObst31,
            urdfObst32,
            urdfObst33,
            urdfObst34
        )
        env.add_obstacle(urdfObst2)
        env.add_obstacle(urdfObst3)
        env.add_obstacle(urdfObst4)
        env.add_obstacle(urdfObst5)
        env.add_obstacle(urdfObst6)
        env.add_obstacle(urdfObst7)
        env.add_obstacle(urdfObst8)
        env.add_obstacle(urdfObst9)
        env.add_obstacle(urdfObst10)
        env.add_obstacle(urdfObst11)
        env.add_obstacle(urdfObst12)
        env.add_obstacle(urdfObst13)
        env.add_obstacle(urdfObst14)
        env.add_obstacle(urdfObst15)
        env.add_obstacle(urdfObst16)
        env.add_obstacle(urdfObst17)
        env.add_obstacle(urdfObst18)
        env.add_obstacle(urdfObst19)
        env.add_obstacle(urdfObst20)
        env.add_obstacle(urdfObst21)
        env.add_obstacle(urdfObst22)
        env.add_obstacle(urdfObst23)
        env.add_obstacle(urdfObst24)
        env.add_obstacle(urdfObst25)
        env.add_obstacle(urdfObst26)
        env.add_obstacle(urdfObst27)
        env.add_obstacle(urdfObst28)
        env.add_obstacle(urdfObst29)
        env.add_obstacle(urdfObst30)
        env.add_obstacle(urdfObst31)
        env.add_obstacle(urdfObst32)
        env.add_obstacle(urdfObst33)
        env.add_obstacle(urdfObst34)
    history = []
    for i in range(n_steps):
        ob, _, _, _ = env.step(action)
        if ob['robot_0']['joint_state']['steering'] > 0.2:
            action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)
