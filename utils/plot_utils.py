import math
import pathlib
import sys

import matplotlib.pyplot as plt
import numpy as np

import utils.specs_utils as specs

# TODO: might remove this
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):
    outline = np.array([[-specs.BACKTOWHEEL,
                         (specs.LENGTH - specs.BACKTOWHEEL),
                         (specs.LENGTH - specs.BACKTOWHEEL),
                         -specs.BACKTOWHEEL,
                         -specs.BACKTOWHEEL],
                        [specs.WIDTH / 2,
                         specs.WIDTH / 2,
                         -specs.WIDTH / 2,
                         -specs.WIDTH / 2,
                         specs.WIDTH / 2]])

    fr_wheel = np.array([[specs.WHEEL_LEN,
                          -specs.WHEEL_LEN,
                          -specs.WHEEL_LEN,
                          specs.WHEEL_LEN,
                          specs.WHEEL_LEN],
                         [-specs.WHEEL_WIDTH - specs.TREAD,
                          -specs.WHEEL_WIDTH - specs.TREAD,
                          specs.WHEEL_WIDTH - specs.TREAD,
                          specs.WHEEL_WIDTH - specs.TREAD,
                          -specs.WHEEL_WIDTH - specs.TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += specs.WB
    fl_wheel[0, :] += specs.WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")


def plot_circle(x, y, radius, color="-b"):
    xl = []
    yl = []
    for rad in np.linspace(0, math.pi*2, 60):
        xl.append(x + radius * math.cos(rad))
        yl.append(y + radius * math.sin(rad))
    plt.fill(xl, yl, "c")
    plt.plot(xl, yl, color)
