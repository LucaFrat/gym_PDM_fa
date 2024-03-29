import matplotlib as mpl
import matplotlib.pyplot as plt


def environ(show_animation=False):
    """
    Draw the map of static obstacles with their circular avoidance regions.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111)

    obstacleList = []
    h_inline = 40
    thick = 1
    h_perimeter = 78
    h_open = 10
    radi = 3
    obst_color = 'red'

    # wall visual drawing
    rett_points = [(-thick/2, -h_inline/2 + 2.5),  # Vertical wall
                   (-3*thick/2-h_inline/2 - 5, -h_inline/2 + 2.5),  # Vertical wall
                   (+thick/2+h_inline/2 + 5, -h_inline/2 + 2.5),  # Vertical wall
                   (-h_perimeter/2, -h_perimeter/2+h_open),  # Vertical wall
                   (h_perimeter/2-thick, -h_perimeter/2),  # Vertical wall
                   (-h_perimeter/2, h_perimeter/2-thick),  # Horizontal wall
                   (-h_perimeter/2, -h_perimeter/2),  # Horizontal wall
                   (-6.5, -18),  # car
                   (-6.5, -12),  # car
                   (1.5, 6),  # car
                   (19.5, 10)]  # car

    circle_centers = [(rett_points[0][0]+thick/2, rett_points[0][1]),
                      (rett_points[1][0]+thick/2, rett_points[1][1]),
                      (rett_points[2][0]+thick/2, rett_points[2][1]),
                      (rett_points[3][0]+thick/2, rett_points[3][1]),
                      (rett_points[4][0]+thick/2, rett_points[4][1]),
                      (rett_points[5][0],         rett_points[5][1]+thick/2),
                      (rett_points[6][0],         rett_points[6][1]+thick/2),
                      (rett_points[7][0]+2.5,     rett_points[7][1]+1.5),
                      (rett_points[8][0]+2.5,     rett_points[8][1]+1.5),
                      (rett_points[9][0]+2.5,     rett_points[9][1]+1.5),
                      (rett_points[10][0]+2.5,    rett_points[10][1]+1.5)]

    rects = [mpl.patches.Rectangle(rett_points[0],
                                   thick,
                                   h_inline,
                                   color=obst_color),
             mpl.patches.Rectangle(rett_points[1],
                                   thick,
                                   h_inline,
                                   color=obst_color),
             mpl.patches.Rectangle(rett_points[2],
                                   thick,
                                   h_inline,
                                   color=obst_color),
             mpl.patches.Rectangle(rett_points[3],
                                   thick,
                                   h_perimeter-h_open,
                                   color=obst_color),
             mpl.patches.Rectangle(rett_points[4],
                                   thick,
                                   h_perimeter-h_open,
                                   color=obst_color),
             mpl.patches.Rectangle(rett_points[5],
                                   h_perimeter,
                                   thick,
                                   color=obst_color),
             mpl.patches.Rectangle(rett_points[6],
                                   h_perimeter,
                                   thick,
                                   color=obst_color)]

    # draw cirlce regions based on the obstacles dimensions
    fill = True
    for i, orig_circ in enumerate(circle_centers):
        length = 0
        if i < 3:
            while length <= h_inline:
                ax.add_patch(mpl.patches.Circle((orig_circ[0],
                                                 orig_circ[1]+length),
                                                radius=radi,
                                                fill=fill))
                obstacleList.append((orig_circ[0], orig_circ[1]+length, radi))
                length += 3*radi/2
        elif i < 5:
            while length <= h_perimeter-h_open:
                ax.add_patch(mpl.patches.Circle((orig_circ[0],
                                                 orig_circ[1]+length),
                                                radius=radi,
                                                fill=fill))
                obstacleList.append((orig_circ[0], orig_circ[1]+length, radi))
                length += 3*radi/2
        elif i < 7:
            while length <= h_perimeter:
                ax.add_patch(mpl.patches.Circle((orig_circ[0]+length,
                                                 orig_circ[1]),
                                                radius=radi,
                                                fill=fill))
                obstacleList.append((orig_circ[0]+length, orig_circ[1], radi))
                length += 3*radi/2
        else:
            ax.add_patch(mpl.patches.Circle((orig_circ[0],
                                             orig_circ[1]),
                                            radius=4,
                                            fill=fill))
            obstacleList.append((orig_circ[0], orig_circ[1], 4))

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
