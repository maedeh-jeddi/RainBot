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
from rosidl_runtime_py.utilities import get_message
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
        # Topics are ready only once a message has ARRIVED on each --
        # see _topics_ready for why a publisher count is not enough.
        self._topic_seen = set()
        self._topic_subs = {}
        self._tf_seen = set()

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
        """Every requested transform, not just the first.

        --tf is repeatable so one gate can wait on a whole fleet localizing.
        Each pair is remembered once it first resolves, so a transform that
        blinks does not send the gate back to the start.
        """
        if self.tf_ok:
            return True
        for pair in self.args.tf:
            key = tuple(pair)
            if key in self._tf_seen:
                continue
            try:
                self.buf.lookup_transform(pair[0], pair[1], rclpy.time.Time())
            except Exception:
                return False
            self._tf_seen.add(key)
            self.get_logger().info(
                f'TF {pair[0]}->{pair[1]} available '
                f'(t+{time.time() - self.t0:.1f}s)')
        self.tf_ok = True
        return True

    def _topics_ready(self):
        """A topic counts as ready when a MESSAGE has actually arrived on it.

        This used to be `count_publishers(t) > 0`, which is cheap and wrong.
        ROS 2's graph keeps announcing publishers belonging to participants that
        have died until their lease expires, and this project kills its
        processes rather than shutting them down, so the count is haunted:
        measured, /r1/nav_ready reported "Publisher count: 2" with both entries
        showing _NODE_NAME_UNKNOWN_ while r1's nav_bringup had never succeeded
        and never created a publisher at all.

        That is not a cosmetic difference. The gates that guard the arms and the
        mission are the fleet's only "everything is really up" check, and a
        ghost satisfied them: the mission was released with r1 dead, then
        dispatched it an errand it could not drive.

        Waiting for a real message cannot be faked by a stale announcement. The
        type is looked up from the graph so the gate still does not need to know
        it in advance.
        """
        ready = True
        for t in self.args.topic:
            if t in self._topic_seen:
                continue
            ready = False
            if t not in self._topic_subs:
                types = dict(self.get_topic_names_and_types()).get(t)
                if not types:
                    continue                 # not advertised yet
                try:
                    msg_cls = get_message(types[0])
                except Exception:            # type not resolvable yet
                    continue
                # BEST_EFFORT so this matches a publisher of either
                # reliability; a RELIABLE writer still satisfies it.
                qos = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
                self._topic_subs[t] = self.create_subscription(
                    msg_cls, t, lambda _m, k=t: self._topic_seen.add(k), qos)
        return ready

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
    # Repeatable: a fleet gate needs map -> r1/base_link AND map -> r2/base_link
    # AND map -> r3/base_link, not just the first of them.
    p.add_argument('--tf', nargs=2, action='append', default=[],
                   metavar=('TARGET', 'SOURCE'))
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
