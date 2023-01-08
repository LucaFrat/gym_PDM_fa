"""

Path tracking simulation with iterative linear model predictive control for speed and steer control

author: Atsushi Sakai (@Atsushi_twi)

"""
import pathlib
import sys

import cvxpy
import numpy as np

import cubic_spline_planner

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 5  # horizon length

# mpc parameters
R = np.diag([0.0, 0.0])  # input cost matrix
Rd = np.diag([0.0, 0.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.0, 0.0])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 1.0  # max simulation time

# iterative paramter
MAX_ITER = 1  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.03  # [s] time tick

TREAD = 1.5
WB = 2.7  # [m]

MAX_STEER = 0.6487  # np.deg2rad(45.0)  # maximum steering angle [rad]
MIN_STEER = -MAX_STEER  # np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 38  # maximum speed [m/s]
MIN_SPEED = -7.0  # minimum speed [m/s]


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, steering=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.steering = steering
        self.predelta = None


def pi_2_pi(angle):
    while angle > np.pi:
        angle = angle - 2.0 * np.pi

    while angle < -np.pi:
        angle = angle + 2.0 * np.pi

    return angle


def get_linear_model_matrix(yaw, steering):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0

    B = np.zeros((NX, NU))
    B[0, 0] = np.cos(yaw) * DT
    B[1, 0] = np.sin(yaw) * DT
    B[2, 0] = np.sin(steering) / WB * DT
    B[3, 1] = DT

    return A, B


def update_state(state, velocity, yaw_rate):

    # input check
    velocity = np.clip(velocity, MIN_SPEED, MAX_SPEED)

    state.x += velocity * np.cos(state.yaw) * DT
    state.y += velocity * np.sin(state.yaw) * DT
    state.yaw += velocity / WB * np.tan(state.steering) * DT
    state.steering += yaw_rate * DT

    state.steering = np.clip(state.steering, MIN_STEER, MAX_STEER)

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = np.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - np.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, ovel, oyr, xref):
    xbar = xref * 0.0
    xbar[:, 0] = x0

    state = State(*xbar[:, 0])
    for (veli, yri, i) in zip(ovel, oyr, range(1, T + 1)):
        state = update_state(state, veli, yri)
        xbar[:, i] = [state.x, state.y, state.yaw, state.steering]

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, ovel, oyr):
    """
    MPC contorl with updating operational point iteraitvely
    """

    if ovel is None or oyr is None:
        ovel = [0.0] * T
        oyr = [0.0] * T

    for i in range(MAX_ITER):
        print("STEP: ", i)
        xbar = predict_motion(x0, ovel, oyr, xref)
        povel, poyr = ovel[:], oyr[:]
        ovel, oyr, ox, oy, oyaw, osteer = linear_mpc_control(
            xref, xbar, x0, dref)

        du = sum(abs(ovel - povel)) + \
            sum(abs(oyr - poyr))  # calc u change value
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return ovel, oyr


def linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B = get_linear_model_matrix(xbar[2, t], xbar[3, t])

        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]

        if t < (T - 1):
            # pay if big difference from an input and the next: high consumption
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(x[3, t + 1] - x[3, t])
                            <= MAX_DSTEER * DT]

    # normal error state cost, last horizon step
    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [u[0, :] <= MAX_SPEED]
    constraints += [u[0, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(x[3, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.OSQP, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        oyaw = get_nparray_from_matrix(x.value[2, :])
        osteer = get_nparray_from_matrix(x.value[3, :])
        ovel = get_nparray_from_matrix(u.value[0, :])
        oyr = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        ovel, oyr, ox, oy, oyaw, osteer = None, None, None, None, None, None

    return ovel, oyr, ox, oy, oyaw, osteer


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref


def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = np.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    if isgoal and isstop:
        return True

    return False


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = np.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= np.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= np.pi / 2.0:
            yaw[i + 1] -= np.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -np.pi / 2.0:
            yaw[i + 1] += np.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def get_forward_course(dl):
    ax = [0.0, 8.0]  # [8.0, 12.0, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
    ay = [0.0, 8.0]  # , 12.0, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck
