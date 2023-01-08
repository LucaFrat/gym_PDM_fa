"""

Path tracking simulation with iterative linear model predictive control for speed and steer control

author: Atsushi Sakai (@Atsushi_twi)

"""
import pathlib
import sys

import cvxpy
import numpy as np

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

NX = 4  # x = x, y, yaw, steering
NU = 2  # u = [velocity, yaw_rate]
T = 20  # horizon length

# mpc parameters
R = np.diag([1, 1])  # input cost matrix
Rd = np.diag([10, 10])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 1.0, 1.0])  # state cost matrix
Qf = Q  # state final matrix

# iterative paramter
MAX_ITER = 1  # Max iteration
DU_TH = 0.1  # iteration finish param

DT = 0.03  # [s] time tick

WB = 2.7  # [m]

MAX_STEER = 0.6487  # np.deg2rad(45.0)  # maximum steering angle [rad]
MIN_STEER = -MAX_STEER  # np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 38  # maximum speed [m/s]
MIN_SPEED = -7.0  # minimum speed [m/s]
MAX_ACCEL = 27


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, steering=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.steering = steering


def get_linear_model_matrix(yaw, steering):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0

    B = np.zeros((NX, NU))
    B[0, 0] = np.cos(yaw) * DT
    B[1, 0] = np.sin(yaw) * DT
    B[2, 0] = np.tan(steering) / WB * DT
    B[3, 1] = DT

    return A, B


def update_state(state, velocity, yaw_rate):

    state.x += velocity * np.cos(state.yaw) * DT
    state.y += velocity * np.sin(state.yaw) * DT
    state.yaw += velocity * np.tan(state.steering) / WB * DT
    state.steering += yaw_rate * DT

    state.steering = np.clip(state.steering, MIN_STEER, MAX_STEER)

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def predict_motion(x0, ovel, oyr, xref):
    xbar = xref * 0.0
    xbar[:, 0] = x0

    state = State(*x0)
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
        xbar = predict_motion(x0, ovel, oyr, xref)
        povel, poyr = ovel[:], oyr[:]
        ovel, oyr, ox, oy, oyaw, osteer = linear_mpc_control(
            xref, xbar, x0, dref)

        du = sum(abs(ovel - povel)) + \
            sum(abs(oyr - poyr))  # calc u change value
        if du <= DU_TH:
            break
    else:
        print("== Max iterations reached")

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
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t])
                            <= MAX_ACCEL * DT]

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
