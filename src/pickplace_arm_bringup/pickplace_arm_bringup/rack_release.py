#!/usr/bin/env python3
"""Break the startup welds between every robot's gripper and every rack.

WHY THIS EXISTS AT ALL
----------------------
The gz DetachableJoint plugin creates its joint the instant the child model
appears, with nobody asking for it. Each robot carries one plugin per graspable
model, so the moment rack_red spawns it is welded to EVERY robot in the world,
wherever that robot happens to be standing -- typically tens of metres away.
With three robots and three racks that is nine joints, all of them wrong.

They are not harmless. A welded rack is rigidly carried at whatever relative
pose it had when the joint was made, so it sweeps through the building as the
robot drives -- through walls, through furniture -- and every one of those
contacts is fed back through the joint into the chassis. Observed directly:
three robots dispatched to three tables, all three racks dragged off, one
finishing 6.4 m from its table on the floor and one riding 0.87 m in the air,
and all three robots wedged with "Collision Ahead" and "Failed to make progress"
before any of them got anywhere.

WHY IT IS A NODE AND NOT NINE `ros2 topic pub --once` CALLS
-----------------------------------------------------------
That is what it was first, and the nine calls all logged "publishing #1"
successfully while not one weld broke. `--once` publishes and tears the
publisher down immediately; `-w 1` waits for a matching SUBSCRIPTION but not for
the sample to be delivered, and nine of them starting at the same instant into a
busy machine is exactly when that gap bites. The same command run by hand,
sequentially, a few minutes later worked every time -- which is the signature of
a delivery race rather than a wrong topic.

So: publish REPEATEDLY, from a process that stays alive long enough for the
middleware to flush, and keep going for long enough to cover the window where a
rack may still be arriving. A detach addressed at a rack that does not exist yet
is consumed and does nothing, so a single well-timed message is not enough on
its own -- repetition covers both failure modes with one mechanism.

Exits 0 whatever happens. A supervisor that can brick a launch is worse than the
failure it guards against, which is the rule wait_for.py and nav_bringup.py
follow too.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Empty


class RackRelease(Node):
    def __init__(self, robots, models, duration, rate):
        super().__init__('rack_release')
        self.duration = duration
        self.rate = rate
        self.pubs = {}
        for ns in robots:
            for model in models:
                topic = f'/{ns}/{model}/detach'
                self.pubs[topic] = self.create_publisher(Empty, topic, 10)
        self.get_logger().info(
            f'[release] {len(self.pubs)} welds to break '
            f'({len(robots)} robots x {len(models)} racks), '
            f'publishing for {duration:.0f}s at {rate:.0f} Hz')

    def run(self):
        # Wait for the bridge to subscribe before the first send. This is not a
        # substitute for repeating -- it just stops the first few messages being
        # thrown away before anything is listening.
        deadline = time.time() + 10.0
        while time.time() < deadline:
            if all(p.get_subscription_count() > 0 for p in self.pubs.values()):
                break
            time.sleep(0.2)
        unmatched = [t for t, p in self.pubs.items()
                     if p.get_subscription_count() == 0]
        if unmatched:
            self.get_logger().warn(
                f'[release] {len(unmatched)} detach topics have no subscriber '
                f'(is the ros_gz bridge up?): {unmatched[:3]}')

        msg = Empty()
        sent = 0
        end = time.time() + self.duration
        while time.time() < end:
            for pub in self.pubs.values():
                pub.publish(msg)
                sent += 1
            time.sleep(1.0 / self.rate)
        self.get_logger().info(
            f'[release] sent {sent} detach messages across {len(self.pubs)} topics')


def main(argv=None):
    argv = remove_ros_args(args=(argv if argv is not None else sys.argv))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot', action='append', default=[],
                    help='robot namespace; repeat for each')
    ap.add_argument('--model', action='append', default=[],
                    help='graspable model name; repeat for each')
    ap.add_argument('--duration', type=float, default=15.0)
    ap.add_argument('--rate', type=float, default=2.0)
    args = ap.parse_args(argv[1:])
    if not args.robot or not args.model:
        print('rack_release: nothing to do (no --robot or no --model)')
        return 0

    rclpy.init()
    node = RackRelease(args.robot, args.model, args.duration, args.rate)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
