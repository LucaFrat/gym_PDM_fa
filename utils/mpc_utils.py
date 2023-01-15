import math
import pathlib
import sys

import cvxpy
import numpy as np

import utils.specs_utils as specs

# TODO: might remove this
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))


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


def get_linear_model_matrix(v, phi, delta):
    A = np.zeros((specs.NX, specs.NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = specs.DT * math.cos(phi)
    A[0, 3] = - specs.DT * v * math.sin(phi)
    A[1, 2] = specs.DT * math.sin(phi)
    A[1, 3] = specs.DT * v * math.cos(phi)
    A[3, 2] = specs.DT * math.tan(delta) / specs.WB

    B = np.zeros((specs.NX, specs.NU))
    B[2, 0] = specs.DT
    B[3, 1] = specs.DT * v / (specs.WB * math.cos(delta) ** 2)

    C = np.zeros(specs.NX)
    C[0] = specs.DT * v * math.sin(phi) * phi
    C[1] = - specs.DT * v * math.cos(phi) * phi
    C[3] = - specs.DT * v * delta / (specs.WB * math.cos(delta) ** 2)

    return A, B, C


def update_state(state, a, delta):

    # input check
    if delta >= specs.MAX_STEER:
        delta = specs.MAX_STEER
    elif delta <= -specs.MAX_STEER:
        delta = -specs.MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * specs.DT
    state.y = state.y + state.v * math.sin(state.yaw) * specs.DT
    state.yaw = state.yaw + state.v / specs.WB * math.tan(delta) * specs.DT
    state.v = state.v + a * specs.DT

    if state.v > specs.MAX_SPEED:
        state.v = specs.MAX_SPEED
    elif state.v < specs.MIN_SPEED:
        state.v = specs.MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, specs.T + 1)):
        state = update_state(state, ai, di)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od, origin_obst):
    """
    MPC contorl with updating operational point iteraitvely
    """

    if oa is None or od is None:
        oa = [0.0] * specs.T
        od = [0.0] * specs.T

    for i in range(specs.MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(
            xref, xbar, x0, dref, origin_obst)

        du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
        if du <= specs.DU_TH:
            break
    else:
        print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref, origin_obst):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    m = []
    for j in range(origin_obst.shape[0]):
        m_obst = []
        for i in range(xbar.shape[1]):
            m_obst.append([(origin_obst[j][0]-xbar[0, i]) /
                          (origin_obst[j][1]-xbar[1, i]), 1])
        m.append(m_obst)

    x = cvxpy.Variable((specs.NX, specs.T + 1))
    u = cvxpy.Variable((specs.NU, specs.T))

    cost = 0.0
    constraints = []

    for t in range(specs.T):
        cost += cvxpy.quad_form(u[:, t], specs.R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], specs.Q)

        A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t])

        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        if t < specs.T - 1:
            # penalize differences between successive inputs: high consumption
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], specs.Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t])
                            <= specs.MAX_DSTEER * specs.DT]

        for j in range(origin_obst.shape[0]):
            if t < specs.T - 4 \
                and (np.abs(origin_obst[j][1]-xbar[1, t+1]) <= 10
                     or np.abs(origin_obst[j][1]-xbar[1, t]) <= 10):
                constraints += [np.sign(origin_obst[j][1]-xbar[1, t])
                                * (m[j][t] @ (x[:2, t+4] - origin_obst[j]))
                                <= 0.00004]

    # normal error state cost, last horizon step
    cost += cvxpy.quad_form(xref[:, specs.T] - x[:, specs.T], specs.Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= specs.MAX_SPEED]
    constraints += [x[2, :] >= specs.MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= specs.MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= specs.MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    if oa is None or odelta is None:
        oa = [0.0] * specs.T
        odelta = [0.0] * specs.T

    return oa, odelta, ox, oy, oyaw, ov
