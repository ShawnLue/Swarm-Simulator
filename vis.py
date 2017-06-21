import matplotlib.pyplot as pyplot
import matplotlib.colors as colors
import matplotlib.cm as cmx
from matplotlib.patches import Rectangle, Circle


def get_cmap(N):
    '''distinct RGB color'''
    color_norm = colors.Normalize(vmin=0, vmax=N-1)
    scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv')

    def map_index_to_rgb_color(index):
        return scalar_map.to_rgba(index)
    return map_index_to_rgb_color


def visualize_traj_dynamic(ws_model, X, U, goal, time=None, name=None):
    figure = pyplot.figure()
    ax = figure.add_subplot(1, 1, 1)
    cmap = get_cmap(len(X))
    # plot obstacles
    for hole in ws_model['circular_obstacles']:
        srec = Rectangle((hole[0] - hole[2], hole[1] - hole[2]),
                         2 * hole[2], 2 * hole[2],
                         facecolor='red',
                         fill=True, alpha=1)
        ax.add_patch(srec)
    # plot traj
    for i in range(len(X)):
        # plot robot
        robot = Circle((X[i][0], X[i][1]), radius=ws_model['robot_radius'],
                       facecolor=cmap(i), edgecolor='black', linewidth=1.0, ls='solid',
                       alpha=1, zorder=2)
        ax.add_patch(robot)
        # plot velocity
        ax.arrow(X[i][0], X[i][1], U[i][0], U[i][1], head_width=0.05, head_length=0.1, fc=cmap(i), ec=cmap(i))
        # plot index of robot
        ax.text(X[i][0] - 0.1, X[i][1] - 0.1, r'$%s$' % i, fontsize=10, fontweight='bold', zorder=3)
        # plot goal position
        ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize=5, linewidth=3.0)
    if time:
        ax.text(2, 5.5, r'$t=%.1f s$' % time, fontsize=20, fontweight='bold')
    # set axes
    ax.set_aspect('equal')
    ax.set_xlim(-1.0, 6.0)
    ax.set_ylim(-1.0, 6.0)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid('on')
    if name:
        pyplot.savefig(name, dpi=200)
    return figure
