import gym
from prius import Prius
import numpy as np
import math
from MotionPlanningGoal.dynamicSubGoal import DynamicSubGoal
import our_MPC 

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


def run_prius(n_steps=500, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel")
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.03, robots=robots, render=render
    )

    #action = np.array([1, 0.4])
    pos0 = np.array([0.0, 0.0, 0.0])
    
    ob = env.reset(pos=pos0)
    #print(f"Initial observation : {ob}")

    history = []
    action = np.array([0.0, 0.0])


    state = State(x=pos0[0], y=pos0[1], yaw=pos0[2], v=0.0)


    splineDict = {'degree': 2, 'controlPoints': [[8.0, 8.0, 0.0], [8.0, 8.0, 0.0], [8.0, 8.0, 0.0]], 'duration': 10}
    splineGoalDict = {
    "weight": 1.0, "is_primary_goal": True, 'indices': [0, 1, 2], 'parent_link': 0, 'child_link': 3,
    'trajectory': splineDict, 'epsilon': 0.08, 'type': "splineSubGoal", 
    }
    splineGoal = DynamicSubGoal(name="goal3", content_dict=splineGoalDict)
    env.add_goal(splineGoal)


    #dl = 0.1
    #cx, cy, cyaw, ck = our_MPC.get_forward_course(dl)
    #sp = our_MPC.calc_speed_profile(cx, cy, cyaw, our_MPC.TARGET_SPEED)
    #target_ind, _ = our_MPC.calc_nearest_index(state, cx, cy, cyaw, 0)
    #cyaw = our_MPC.smooth_yaw(cyaw)
    oa, od = None, None
    xref = np.ones((our_MPC.NX, our_MPC.T+1))*8.0
    xref[2:,:] = 0.0

    dref = np.zeros((1, our_MPC.T+1))
    delta_max = 0
    steer_max = 0

    delta_prev = 0
    yaw_prev = 0

    for i in range(n_steps):
        
        #xref, target_ind, dref = our_MPC.calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, target_ind)
        
        x0 = [state.x, state.y, state.v, state.yaw] 

        acc, delta, _, _, v_input, yaw_input = our_MPC.iterative_linear_mpc_control(xref, x0, dref, oa, od)

        if delta is not None:
            deltai, acci = delta[0], acc[0]
            

        state = our_MPC.update_state(state, acci, deltai)

        action[1] = action[0] *math.tan(delta[0]) /our_MPC.WB
        action[0] = acc[0]*our_MPC.DT + action[0]     
        
        #action[1] = (state.yaw-yaw_input[1])/our_MPC.DT
        yaw_prev = state.yaw
        delta_prev = delta[0]

        #action[1] = action[0]*math.tan(delta[0]) /our_MPC.WB 
        #action[1] = (delta[0]-delta_prev)/our_MPC.DT

        #if ob['robot_0']['joint_state']['steering'] > our_MPC.MAX_STEER or ob['robot_0']['joint_state']['steering'] < -our_MPC.MAX_STEER:
        #if ob['robot_0']['joint_state']['steering'] > 0.1 or ob['robot_0']['joint_state']['steering'] < -0.1:
        #    action[1] = 0

        print(f"""vel: {action[0]}   yaw_rate: {action[1]}
                  steer: {ob['robot_0']['joint_state']['steering']}""")
        if delta[0] > delta_max:
            delta_max = delta[0]
        if ob['robot_0']['joint_state']['steering'] > steer_max:
            steer_max = ob['robot_0']['joint_state']['steering']
        ob, _, _, _ = env.step(action)

        
        history.append(ob)
    
    print(f"""
              steer_max = {steer_max}
              delta_max = {delta_max}""")
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)