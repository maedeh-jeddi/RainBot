#!/usr/bin/env python3
"""Minimal keyboard teleop for the mobile base — drive it around during a
slam_toolbox mapping session. No external deps (raw termios stdin).

Keys:
  w / s : forward / backward      a / d : turn left / right
  x     : stop                    q     : quit
Speeds ramp with repeated presses; released keys decay to a stop.
"""
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = __doc__
LIN_STEP = 0.05
ANG_STEP = 0.2
LIN_MAX = 0.6
ANG_MAX = 1.2


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.lin = 0.0
        self.ang = 0.0

    def publish(self):
        t = Twist()
        t.linear.x = self.lin
        t.angular.z = self.ang
        self.pub.publish(t)


def get_key(timeout=0.1):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if r:
        return sys.stdin.read(1)
    return ''


def main():
    rclpy.init()
    node = TeleopKey()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            k = get_key()
            if k == 'w':
                node.lin = clamp(node.lin + LIN_STEP, -LIN_MAX, LIN_MAX)
            elif k == 's':
                node.lin = clamp(node.lin - LIN_STEP, -LIN_MAX, LIN_MAX)
            elif k == 'a':
                node.ang = clamp(node.ang + ANG_STEP, -ANG_MAX, ANG_MAX)
            elif k == 'd':
                node.ang = clamp(node.ang - ANG_STEP, -ANG_MAX, ANG_MAX)
            elif k == 'x':
                node.lin = 0.0
                node.ang = 0.0
            elif k == 'q':
                break
            elif k == '':
                # no key: gently decay toward stop so the base doesn't run away
                node.lin *= 0.8
                node.ang *= 0.8
                if abs(node.lin) < 0.02:
                    node.lin = 0.0
                if abs(node.ang) < 0.02:
                    node.ang = 0.0
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.lin = node.ang = 0.0
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
