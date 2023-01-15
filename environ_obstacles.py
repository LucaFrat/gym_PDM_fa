import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


def environ(show_animation=False):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    obstacleList = []
    h_inline = 50
    thick = 1
    h_perimeter = 78
    h_open = 10
    radi = 3
    obst_color = 'red'

    rett_points = [(-thick/2, -h_inline/2),  # V
                   (-3*thick/2-h_inline/2, -h_inline/2),  # V
                   (+thick/2+h_inline/2, -h_inline/2),  # V
                   (-h_perimeter/2, -h_perimeter/2+h_open),  # V
                   (h_perimeter/2-thick, -h_perimeter/2),  # V
                   (-h_perimeter/2, h_perimeter/2-thick),  # O
                   (-h_perimeter/2, -h_perimeter/2)]  # O

    circle_centers = [(rett_points[0][0]+thick/2, rett_points[0][1]),
                      (rett_points[1][0]+thick/2, rett_points[1][1]),
                      (rett_points[2][0]+thick/2, rett_points[2][1]),
                      (rett_points[3][0]+thick/2, rett_points[3][1]),
                      (rett_points[4][0]+thick/2, rett_points[4][1]),
                      (rett_points[5][0], rett_points[5][1]+thick/2),
                      (rett_points[6][0], rett_points[6][1]+thick/2)]

    rects = [mpl.patches.Rectangle(rett_points[0], thick,  h_inline, color=obst_color),
             mpl.patches.Rectangle(
                 rett_points[1], thick,  h_inline, color=obst_color),
             mpl.patches.Rectangle(
                 rett_points[2], thick,  h_inline, color=obst_color),

             mpl.patches.Rectangle(
                 rett_points[3], thick,       h_perimeter-h_open, color=obst_color),
             mpl.patches.Rectangle(
                 rett_points[4], thick,       h_perimeter-h_open, color=obst_color),
             mpl.patches.Rectangle(
                 rett_points[5], h_perimeter, thick,              color=obst_color),
             mpl.patches.Rectangle(rett_points[6], h_perimeter, thick,              color=obst_color)]

    fill = True
    for i, orig_circ in enumerate(circle_centers):
        length = 0
        if i < 3:
            while length <= h_inline:
                ax.add_patch(mpl.patches.Circle(
                    (orig_circ[0], orig_circ[1]+length), radius=radi, fill=fill))
                obstacleList.append((orig_circ[0], orig_circ[1]+length, radi))
                length += 3*radi/2
        elif i < 5:
            while length <= h_perimeter-h_open:
                ax.add_patch(mpl.patches.Circle(
                    (orig_circ[0], orig_circ[1]+length), radius=radi, fill=fill))
                obstacleList.append((orig_circ[0], orig_circ[1]+length, radi))
                length += 3*radi/2
        else:
            while length <= h_perimeter:
                ax.add_patch(mpl.patches.Circle(
                    (orig_circ[0]+length, orig_circ[1]), radius=radi, fill=fill))
                obstacleList.append((orig_circ[0]+length, orig_circ[1], radi))
                length += 3*radi/2

    for rect in rects:
        ax.add_patch(rect)

    plt.grid()
    plt.xlim([-50, 50])
    plt.ylim([-50, 50])

    if show_animation:
        plt.show()

    return obstacleList


if __name__ == 'main':
    environ()
