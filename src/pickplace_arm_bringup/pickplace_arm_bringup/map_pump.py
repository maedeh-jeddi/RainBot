"""Re-publish the shared /map periodically while the fleet is coming up.

WHY THIS EXISTS
---------------
/map is published once, latched (TRANSIENT_LOCAL), by a single map_server that
every robot's amcl and both of its costmaps subscribe to. A latched topic is
supposed to make that safe for late joiners, and most of the time it does. It
does not do it reliably here, and the fleet cannot come up without it:

    r1: (never even logged "Subscribed to map topic")
    r2: Received a 540 X 1160 map
    r3: Received a 540 X 1160 map     -- and still ended up not active

An amcl that has not got a map never leaves configuring, so its whole stack
stays inactive, nav_bringup burns all four of its attempts on it, and the
mission gate waits on a robot that was never going to arrive. Measured across
this project's runs it lands on a different robot each time, or none.

nav_bringup makes it worse by design: when a stack stalls it RESETs and retries,
which tears amcl back down to unconfigured and builds a NEW map subscription on
the way back up. Each rebuild is another chance to miss the one latched sample,
so the recovery path shares the failure it is recovering from.

THE FIX IS TO STOP RELYING ON A SINGLE DELIVERY. This node takes the map once
and re-publishes that identical message every couple of seconds during bring-up,
so a subscriber that appears -- or reappears after a reset -- at any moment gets
one within seconds.

THIS REQUIRES amcl's first_map_only TO BE TRUE, and that is not optional.
nav2_amcl's map callback does NOT ignore a repeat: handleMapMessage frees the
existing map, rebuilds the likelihood field and RE-SEEDS THE PARTICLE FILTER
from the initial pose. Pumping the map at a robot that has been driving for ten
minutes would throw away its localization. With first_map_only set, amcl keeps
the first map it received and drops the rest, which is exactly the behaviour
this node needs from it.

IT STOPS ON ITS OWN. Every subscriber that matters exists within the first few
minutes, and re-processing a 540x1160 grid in three robots' static layers is not
free, so the pump runs for --duration and then goes quiet. The publisher stays
alive afterwards, still TRANSIENT_LOCAL, so anything that joins later still has
a latched copy to collect.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid

# Matches map_server's own publisher, and what every subscriber here expects.
MAP_QOS = QoSProfile(depth=1,
                     history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MapPump(Node):
    def __init__(self, period, duration):
        super().__init__('map_pump')
        self._map = None
        self._sent = 0
        self._duration = duration
        self._done = False
        # Just long enough to cover the subscribers that already exist.
        self._burst = 20.0
        self._last_count = 0
        self._start = self.get_clock().now()
        self._pub = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
        self._sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, MAP_QOS)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'[map_pump] re-publishing /map every {period:.1f}s for '
            f'{duration:.0f}s once the map arrives')

    def _on_map(self, msg):
        # Our own re-publications come back here too; only the first matters.
        if self._map is None:
            self._map = msg
            self.get_logger().info(
                f'[map_pump] got the map ({msg.info.width}x{msg.info.height}) '
                f'-- pumping it to any subscriber that missed it')

    def _tick(self):
        """Publish only when a NEW subscriber has appeared.

        The first version simply re-published every 2 s, and that is too blunt:
        /map is not only read by amcl. Every costmap's static layer re-processes
        it, and so does rviz's Map display, which measured 144% of a core under
        a 2 Hz pump against 70% without one -- on the machine whose bring-up
        deadlines this node exists to protect. Pumping harder made bring-up
        slower.

        A subscriber count that has gone UP is exactly the event that matters:
        it means an amcl (or a costmap) has just subscribed, or re-subscribed
        after nav_bringup reset its stack, and that is precisely who might have
        missed the latched sample. Steady state costs nothing at all.
        """
        if self._map is None or self._done:
            return
        elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed > self._duration:
            self._done = True
            self.get_logger().info(
                f'[map_pump] done after {self._sent} republishes; the latched '
                f'copy stays available')
            return
        count = self._pub.get_subscription_count()
        # A short opening burst seeds whoever is already listening, then it goes
        # purely event-driven.
        if elapsed < self._burst or count > self._last_count:
            if count != self._last_count:
                self.get_logger().info(
                    f'[map_pump] /map subscribers {self._last_count} -> {count}'
                    f' -- republishing')
            self._pub.publish(self._map)
            self._sent += 1
        self._last_count = count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--period', type=float, default=2.0)
    ap.add_argument('--duration', type=float, default=300.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = MapPump(args.period, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
