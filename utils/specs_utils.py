import numpy as np

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]

DT = 0.2  # [s] time tick
T = int(3.0/DT)  # horizon length

# mpc parameters
R = np.diag([1, 0.01])  # input cost matrix
Rd = np.diag([0.2, 0.01])  # input difference cost matrix
Q = np.diag([0.2, 0.2, 0.2, 0.1])  # state cost matrix
Qf = Q  # state final matrix

# dynamic obstacle avoidance
AVOID_RADIUS = 3

GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 15  # Search index number


# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 2.7  # [m]

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

show_animation = True
