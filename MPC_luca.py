"""

Path tracking simulation with iterative linear model predictive control for speed and steer control

author: Atsushi Sakai (@Atsushi_twi)

"""
import pathlib
import sys

import cvxpy as cp
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


def predict_motion(x, velocities, yaw_rates):
    xbar = np.zeros((len(x), len(velocities) + 1))
    xbar[:, 0] = x

    for (velocity, yaw_rate, t) in zip(velocities, yaw_rates, range(1, T + 1)):
        A, B = get_linear_model_matrix(*x[2:])
        xbar[:, t] = A @ xbar[:, t - 1] + B @ [velocity, yaw_rate]

    return xbar


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def linear_mpc_control(x0, xref, velocities, yaw_rates):
    """
    linear mpc control

    xref: reference point
    x: initial state
    """

    x = cp.Variable((NX, T + 1))
    u = cp.Variable((NU, T))

    cost = 0.0
    constraints = []

    xbar = predict_motion(x0, velocities, yaw_rates)

    for t in range(T):
        cost += cp.quad_form(u[:, t], R)

        if t != 0:
            cost += cp.quad_form(xref - x[:, t], Q)

        A, B = get_linear_model_matrix(xbar[2, t], xbar[3, t])

        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]

        if t < (T - 1):
            # pay if big difference from an input and the next: high consumption
            cost += cp.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cp.abs(x[3, t + 1] - x[3, t])
                            <= MAX_DSTEER * DT]
            constraints += [cp.abs(u[1, t + 1] - u[1, t])
                            <= MAX_ACCEL * DT]

    # normal error state cost, last horizon step
    cost += cp.quad_form(xref - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [u[0, :] <= MAX_SPEED]
    constraints += [u[0, :] >= MIN_SPEED]
    constraints += [cp.abs(x[3, :]) <= MAX_STEER]

    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.ECOS, verbose=False)

    velocity = yaw_rate = 0
    if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
        velocity = get_nparray_from_matrix(u.value[0, :])
        yaw_rate = get_nparray_from_matrix(u.value[1, :])

    return velocity, yaw_rate
