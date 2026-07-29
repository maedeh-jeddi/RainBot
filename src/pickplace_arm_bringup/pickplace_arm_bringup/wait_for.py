"""Block until the sim stack is actually ready, then exit 0.

WHY THIS EXISTS
---------------
The mission launches used to stage themselves on fixed TimerActions -- props at
12 s, localization at 75 s, nav2 at 95 s, the mission itself at 120 s. Those
numbers were calibrated against a real symptom: /clock jumping BACKWARDS
hundreds of times during startup, which makes AMCL throw
tf2::ExtrapolationException on lidar_link->odom and abort outright (SIGABRT),
stranding the run.

Re-measured, there are TWO different things that both look like "the clock
jumped backwards", and only one of them is a fault:

  1. TRANSPORT REORDERING -- benign, and constant. /clock is BEST_EFFORT over
     UDP, so consecutive messages arrive out of order all the time. Sampled
     live off a healthy sim: 310 backward steps in 2316 messages, every single
     one exactly one 10 ms sim tick (max 20 ms), while the clock advanced 113 s
     net over the same 12 s window. Nothing is wrong; that is just UDP.

  2. A SECOND SIMULATOR -- the real fault. An orphaned gz server left from a
     previous run keeps publishing onto the same /clock, and the two disagree
     by whole seconds. Measured with a duplicate stack running: 144 jump-backs
     continuing to t+74.2 s, which is almost exactly where the old 75 s
     localization timer sat. That is what those timers were really buying
     protection from.

Started clean, with one stack, the sim is ready in seconds: first /clock at
t+3.2 s, TF odom->base_link at t+5.3 s. So the fixed schedule spent ~115 s
waiting for nothing on a healthy machine, and on a dirty one it silently
papered over a process leak instead of surfacing it.

Hence --jump-threshold (default 0.1 s): 5x larger than the worst benign
reordering step, orders of magnitude smaller than a real two-simulator
disagreement. Below it, steps are ignored; above it, the stability window
restarts AND the log names the likely cause. A clean start therefore proceeds
in a few seconds, while a genuinely sick clock still keeps AMCL from coming up
and aborting on tf2::ExtrapolationException.

Every check is bounded by --timeout. On expiry this still exits 0, with a
warning: a stuck probe must never be able to brick a launch that would
otherwise have worked. The stage behind it starts anyway, exactly as the old
unconditional timer would have.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock

import tf2_ros


# /clock is published BEST_EFFORT/VOLATILE by ros_gz_bridge; a RELIABLE
# subscription silently never matches it.
CLOCK_QOS = QoSProfile(depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST)


class WaitFor(Node):
    def __init__(self, args):
        super().__init__('wait_for')
        self.args = args
        self.t0 = time.time()
        self.done = False

        self.clock_seen = False
        self.last_clock = None
        self.stable_since = None
        self.jumps = 0
        self.worst_jump = 0.0

        if args.clock_stable > 0.0:
            self.create_subscription(Clock, '/clock', self._clock_cb, CLOCK_QOS)

        if args.tf:
            self.buf = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.buf, self)
        self.tf_ok = not args.tf

        self.create_timer(0.25, self._tick)

    # --- individual checks ---------------------------------------------------
    def _clock_cb(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        now = time.time()
        if not self.clock_seen:
            self.clock_seen = True
            self.stable_since = now
            self.get_logger().info(f'/clock is publishing (t+{now - self.t0:.1f}s)')
        elif t < self.last_clock - self.args.jump_threshold:
            # A REAL backwards jump. Restart the stability window and say so
            # loudly -- this almost always means an orphaned gz server from a
            # previous run is still alive and fighting this one for /clock.
            self.jumps += 1
            self.stable_since = now
            self.worst_jump = max(self.worst_jump, self.last_clock - t)
            if self.jumps in (1, 10, 50, 200):
                self.get_logger().warn(
                    f'/clock jumped BACKWARDS by {self.last_clock - t:.2f}s '
                    f'({self.jumps} so far). If this persists, a previous run\'s '
                    f'gz server is probably still running: pkill -9 -f "gz sim"')
        # First message has no previous value to compare against.
        self.last_clock = t if self.last_clock is None else max(self.last_clock, t)

    def _clock_ready(self):
        if self.args.clock_stable <= 0.0:
            return True
        if not self.clock_seen or self.stable_since is None:
            return False
        return (time.time() - self.stable_since) >= self.args.clock_stable

    def _tf_ready(self):
        if self.tf_ok:
            return True
        try:
            self.buf.lookup_transform(self.args.tf[0], self.args.tf[1],
                                      rclpy.time.Time())
            self.tf_ok = True
            self.get_logger().info(
                f'TF {self.args.tf[0]}->{self.args.tf[1]} available '
                f'(t+{time.time() - self.t0:.1f}s)')
        except Exception:
            pass
        return self.tf_ok

    def _topics_ready(self):
        for t in self.args.topic:
            if not self.count_publishers(t):
                return False
        return True

    def _services_ready(self):
        names = {n for n, _ in self.get_service_names_and_types()}
        return all(s in names for s in self.args.service)

    def _actions_ready(self):
        # An action server exposes <action>/_action/send_goal as a service, so
        # this needs no action client (and no type import) to detect.
        names = {n for n, _ in self.get_service_names_and_types()}
        return all(f'{a}/_action/send_goal' in names for a in self.args.action)

    def _nodes_ready(self):
        live = set(self.get_node_names())
        return all(n.lstrip('/') in live for n in self.args.node)

    # --- driver --------------------------------------------------------------
    def _tick(self):
        if self.done:
            return
        elapsed = time.time() - self.t0

        checks = (('clock', self._clock_ready()), ('tf', self._tf_ready()),
                  ('topics', self._topics_ready()),
                  ('services', self._services_ready()),
                  ('actions', self._actions_ready()),
                  ('nodes', self._nodes_ready()))

        if all(ok for _, ok in checks):
            self.done = True
            extra = (f' ({self.jumps} real clock jump-backs, worst '
                     f'{self.worst_jump:.2f}s)' if self.jumps else '')
            self.get_logger().info(
                f'[{self.args.label}] ready after {elapsed:.1f}s{extra}')
            raise SystemExit(0)

        if elapsed >= self.args.timeout:
            self.done = True
            pending = ', '.join(n for n, ok in checks if not ok)
            self.get_logger().warn(
                f'[{self.args.label}] TIMEOUT after {elapsed:.1f}s waiting on: '
                f'{pending}. Continuing anyway.')
            raise SystemExit(0)

        # Progress note roughly every 5 s so a long wait is never silent.
        if int(elapsed * 4) % 20 == 0 and elapsed >= 5.0:
            pending = ', '.join(n for n, ok in checks if not ok)
            self.get_logger().info(
                f'[{self.args.label}] waiting {elapsed:.0f}s on: {pending}')


def main(argv=None):
    # launch_ros appends its own "--ros-args -r __node:=... --params-file ..."
    # to every Node's arguments. Those must be stripped before argparse sees
    # them, and anything left over tolerated, or the gate dies instantly with
    # "unrecognized arguments" -- which is especially nasty here because
    # OnProcessExit fires on ANY exit, so the launch would sail on with every
    # stage effectively ungated and no obvious sign of it.
    argv = remove_ros_args(args=sys.argv)[1:] if argv is None else argv
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--label', default='wait_for')
    p.add_argument('--clock-stable', type=float, default=0.0,
                   help='require /clock free of REAL backward jumps for this '
                        'many seconds before declaring ready')
    p.add_argument('--jump-threshold', type=float, default=0.1,
                   help='backward step (s) that counts as a real jump. /clock '
                        'is BEST_EFFORT over UDP, so consecutive messages get '
                        'delivered out of order routinely: measured live, every '
                        'backward step was exactly one 10 ms sim tick (max 20 '
                        'ms) while the clock advanced 113 s net over the same '
                        'window. Counting those as resets made this gate block '
                        'forever. A genuine fault -- an orphaned second gz '
                        'server, a sim reset -- moves time by whole seconds, so '
                        '0.1 s separates the two cleanly with 5x margin.')
    p.add_argument('--tf', nargs=2, metavar=('TARGET', 'SOURCE'))
    p.add_argument('--topic', action='append', default=[])
    p.add_argument('--service', action='append', default=[])
    p.add_argument('--action', action='append', default=[])
    p.add_argument('--node', action='append', default=[])
    p.add_argument('--timeout', type=float, default=120.0)
    args, _unknown = p.parse_known_args(argv)

    rclpy.init()
    node = WaitFor(args)
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = exc.code or 0
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
