#!/usr/bin/env python3
'''
Records the drone path + tool map during a flight and saves a top down plot when you Ctrl+C.
Used this to make the picture in the README and to check how far off the tool estimates are.

    python3 scripts/plot_mission.py                 # saves mission_plot.png
    python3 scripts/plot_mission.py --truth -o docs/mission_plot.png

--truth also draws where the tools ACTUALLY are in the gazebo world (from sim/worlds/tool_field.sdf)
'''

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from spacedrone_interfaces.msg import ToolMap

import matplotlib
matplotlib.use('Agg')  # no window, just save the file
import matplotlib.pyplot as plt

# ground truth from the world file, NED (north, east)
TRUTH = {
    0: ('wrench', 6.0, 3.0),
    3: ('hammer', 11.0, -6.0),
    7: ('drill', 16.0, 7.0),
    12: ('toolbox', 18.0, -2.0),
}

# colors
PATH_COLOR = '#2a78d6'
EST_COLOR = '#eb6834'
TRUTH_COLOR = '#52514e'
TEXT_COLOR = '#0b0b0b'
BG = '#fcfcfb'


class Recorder(Node):
    def __init__(self):
        super().__init__('mission_plotter')
        self.path = []
        self.tools = {}
        self.create_subscription(PoseStamped, '/spacedrone/vehicle_pose', self.pose_cb, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ToolMap, '/spacedrone/tool_map', self.map_cb, qos)
        self.get_logger().info('recording... ctrl+c to save the plot')

    def pose_cb(self, msg):
        p = msg.pose.position
        # only keep a point every ~10cm or the list gets huge
        if self.path and math.hypot(p.x - self.path[-1][0], p.y - self.path[-1][1]) < 0.1:
            return
        self.path.append((p.x, p.y))

    def map_cb(self, msg):
        self.tools = {t.tag_id: t for t in msg.tools}


def make_plot(path, tools, show_truth, out):
    fig, ax = plt.subplots(figsize=(7, 8), facecolor=BG)
    ax.set_facecolor(BG)

    # plot with east on x and north on y so it looks like a map
    if path:
        ax.plot([p[1] for p in path], [p[0] for p in path], color=PATH_COLOR, linewidth=2,
                label='flight path', zorder=2)

    ax.scatter([0], [0], marker='s', s=80, color=TEXT_COLOR, zorder=4)
    ax.annotate('home', (0, 0), xytext=(8, -14), textcoords='offset points', color=TEXT_COLOR, fontsize=9)

    if show_truth:
        ax.scatter([t[2] for t in TRUTH.values()], [t[1] for t in TRUTH.values()], s=160,
                   facecolors='none', edgecolors=TRUTH_COLOR, linewidths=1.5,
                   label='actual tool position', zorder=3)

    errors = []
    first = True
    for tid, t in tools.items():
        n, e = t.position.x, t.position.y
        # only label the first one or the legend has 4 copies
        ax.scatter([e], [n], s=60, color=EST_COLOR, edgecolors=BG, linewidths=2, zorder=5,
                   label='estimated (tool registry)' if first else None)
        first = False
        label = t.tool_name
        if show_truth and tid in TRUTH:
            err = math.hypot(n - TRUTH[tid][1], e - TRUTH[tid][2])
            errors.append(err)
            label += f'  ({err * 100:.0f} cm off)'
        ax.annotate(label, (e, n), xytext=(10, 6), textcoords='offset points', color=TEXT_COLOR, fontsize=9)

    ax.set_xlabel('east (m)', color='#52514e')
    ax.set_ylabel('north (m)', color='#52514e')
    ax.set_aspect('equal')
    ax.grid(True, color='#e4e3df', linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_color('#c3c2b7')
    ax.tick_params(colors='#52514e')

    title = 'Autonomous search + tool localization (Gazebo / PX4 SITL)'
    if errors:
        title += f'\nmean error {sum(errors) / len(errors) * 100:.0f} cm over {len(errors)} tools'
    ax.set_title(title, color=TEXT_COLOR, fontsize=11)
    # ax.legend(loc='lower right', frameon=False, fontsize=9)   # sat right on top of the path
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=3, frameon=False, fontsize=9)

    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor=BG)
    print(f'saved {out}  ({len(path)} path points, {len(tools)} tools)')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--out', default='mission_plot.png')
    parser.add_argument('--truth', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    rec = Recorder()
    try:
        rclpy.spin(rec)
    except KeyboardInterrupt:
        pass
    make_plot(rec.path, rec.tools, args.truth, args.out)
    rec.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
