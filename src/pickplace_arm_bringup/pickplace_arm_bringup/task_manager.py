#!/usr/bin/env python3
"""Assigns the three errands, hands out delivery slots and parking vertices, and
draws the whole job in RViz.

WHAT IT DECIDES
    which robot fetches which rack   -- once, at dispatch
    which slot a rack goes in        -- on request, when a robot arrives
    which parking vertex a robot ends on -- on request, when it is done
    when each robot is allowed to leave reception, and when it may use the
    delivery table                   -- see the two locks below

WHY THE LAST TWO ARE HERE AND NOT IN THE ROBOTS. Three robots that leave a 3.46 m
triangle at the same instant, all heading for the same lobby exit, drive into
each other: measured, two of them ended 0.99 m apart, closer than the 1.20 m two
Husky circumscribed radii need, and both wedged with "Collision Ahead" before
either got anywhere. Sent one at a time, all three complete the identical routes.

So this is a coordination problem, and it is solved where coordination belongs --
in the one node that can see all three robots at once. Two rules do it:

  DEPARTURE STAGGER  depart_stagger seconds between departures -- 20 by
                     all three leave together and their three errands overlap
                     end to end. See the constant for what had to change in the
                     controller before that was safe.

  DELIVERY QUEUE     one reserved waiting BEARING per robot on a 3 m circle
                     around the delivery table, so all three make the trip as
                     soon as they are carrying and wait AT the table rather
                     than at their collection benches. Reserved rather than
                     derived from the approach direction, because all three
                     collection tables are south of the lobby and two of the
                     three approaches cross: see DELIVERY_HOLD_BEARINGS in
                     rack_table_layout.

  DELIVERY LOCK      only one robot may be working at the delivery table at a
                     time. Its standoff, the creep in and the arm placement all
                     happen inside a space barely wider than one robot, so two
                     robots trying to use it at once is the departure collision
                     again with a table in the middle.

Neither rule slows the fleet down in any way that matters here: the three
errands are 29, 40 and 40 m long, so the robots are naturally spread out for
almost the whole run, and the locks only bite at the two moments they are all in
the same place.

This is the same shape of answer a traffic-schedule fleet manager gives, just
sized to three robots and one shared table rather than a whole building.

WHAT IT DOES NOT DO is replace the robots' own obstacle avoidance. Each robot's
LIDAR still sees the others and its local costmap still plans around them; that
handles incidental encounters in a corridor, which no schedule can predict.
"""
import math
import os
import sys
import threading
import time

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from pickplace_arm_bringup.fleet_layout import ROBOTS, parking_vertices
from pickplace_arm_bringup import rack_table_layout as RT

# Seconds between one robot leaving reception and the next being dispatched.
#
# TWENTY, AND IT IS A LAUNCH ARGUMENT, NOT A CONSTANT TO EDIT:
#
#     ros2 launch pickplace_arm_bringup hospital_mission.launch.py \
#         depart_stagger:=20.0
#
# This has been 25, then 12, then 0, and back to 20 -- which is the point of
# making it an argument. Zero was tried because the errands are meant to overlap,
# and they still do: the stagger only separates the DEPARTURES, and the routes
# are 29 to 40 m long, so the robots spend almost the whole run spread out
# anyway. What the separation buys is the two moments they are all in one place
# -- leaving the reception triangle, and arriving at the one delivery standoff.
#
# The value below is only the fallback for running this node by hand; the launch
# file passes the real one as a ROS parameter.
DEFAULT_DEPART_STAGGER = float(os.environ.get('FLEET_DEPART_STAGGER', 20.0))

COLOUR_RGBA = {'red': (0.9, 0.05, 0.05, 1.0),
               'green': (0.05, 0.7, 0.05, 1.0),
               'blue': (0.1, 0.5, 0.9, 1.0)}


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        # Overridable per launch: depart_stagger:=<seconds>. See
        # DEFAULT_DEPART_STAGGER for what it is for.
        self.declare_parameter('depart_stagger', DEFAULT_DEPART_STAGGER)
        self.depart_stagger = float(
            self.get_parameter('depart_stagger').value)
        self.robots = [ns for ns, *_ in ROBOTS]
        self.collections = RT.collection_points()
        self.slots = RT.delivery_slots()
        self.parks = parking_vertices()

        self._lock = threading.Lock()
        self._free_slots = list(range(len(self.slots)))
        self._free_parks = list(range(len(self.parks)))
        self._slot_owner = {}
        self._park_owner = {}
        self._delivery_busy = None          # ns currently using the table
        self.status = {ns: ('waiting', '') for ns in self.robots}
        # Robots whose rack reached the delivery table, regardless of how
        # their errand ended afterwards. See _on_status.
        self.delivered = set()
        # Robots that have stopped on the holding ring, and robots whose errand
        # is over either way. Both feed the rendezvous in _claim_slot.
        self.holding = set()
        self.finished = set()
        # Placing order: r1, then r2, then r3 -- fleet order, not arrival order.
        self.place_order = list(self.robots)
        self.assignment = {}

        # LATCHED, AND PUBLISHED ONCE. Sent as ordinary volatile messages this
        # had to be repeated to be sure a still-constructing mission node saw
        # it -- and then the node saw all the copies, so a failed errand was
        # immediately retried against the leftovers in its queue. Transient
        # local durability gives a late subscriber the last message and only
        # the last message, which is exactly the semantics a task assignment
        # wants.
        task_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.task_pubs = {ns: self.create_publisher(String, f'/{ns}/task', task_qos)
                          for ns in self.robots}
        for ns in self.robots:
            self.create_subscription(
                String, f'/{ns}/mission_status',
                lambda m, k=ns: self._on_status(k, m), 10)
            # One service per robot rather than one shared service taking a
            # robot name: std_srvs/Trigger has no request payload, and the
            # service NAME is then what identifies the caller. That keeps the
            # whole protocol inside stock message packages, with no custom
            # interface package to build.
            self.create_service(Trigger, f'/{ns}/claim_slot',
                                lambda req, res, k=ns: self._claim_slot(k, res))
            self.create_service(Trigger, f'/{ns}/claim_park',
                                lambda req, res, k=ns: self._claim_park(k, res))

        # ONE TF listener for the whole fleet, here rather than one per launch
        # gate. The manager is the thing that decides when a robot may be given
        # work, so it is the thing that should check whether that robot can do
        # any: map -> <ns>/base_link exists only once that robot's AMCL is
        # ACTIVE and has fused a scan.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, '/fleet/markers', 1)
        self.board_pub = self.create_publisher(String, '/fleet/status', 10)
        self.create_timer(1.0, self._publish_markers)
        self.create_timer(2.0, self._publish_board)
        self.get_logger().info(
            f'task manager up: {len(self.robots)} robots, '
            f'{len(self.collections)} collection points, {len(self.slots)} slots')

    # --- allocation -----------------------------------------------------------
    def _claim_slot(self, ns, res):
        """Grant the table only when EVERY robot is holding, and then strictly
        in fleet order: r1 places and parks, then r2, then r3.

        TWO CONDITIONS, AND THE FIRST IS A RENDEZVOUS. A robot that reaches the
        holding ring is refused until all the others have reached it too, so
        nobody starts placing while another robot is still driving in. The
        second is the order: the queue used to be first-come-first-served, which
        made the row fill up in whatever sequence the drives happened to finish.

        A ROBOT'S TURN ENDS WHEN IT HAS PARKED, not when it has placed. Releasing
        on 'delivered' let the next robot drive in while the previous one was
        still backing away from the standoff and turning for the lobby -- two
        robots inside the same few metres, which is the collision this lock
        exists to prevent.

        Robots that have already finished -- or failed anywhere along the way --
        are excluded from both conditions, so one failure cannot deadlock the
        rest of the fleet behind a rendezvous that can never complete.
        """
        with self._lock:
            self.holding.add(ns)
            pending = [r for r in self.robots
                       if r not in self.holding and r not in self.finished]
            if pending:
                res.success = False
                res.message = (f'in use: waiting for {", ".join(pending)} to '
                               f'reach the holding ring')
                return res
            turn = next((r for r in self.place_order
                         if r not in self.finished), None)
            if turn is not None and turn != ns:
                res.success = False
                res.message = f'in use by {turn} (placing in fleet order)'
                return res
            if self._delivery_busy not in (None, ns):
                res.success = False
                res.message = f'delivery table in use by {self._delivery_busy}'
                return res
            if not self._free_slots:
                res.success = False
                res.message = 'no free slot'
                return res
            slot = self._free_slots.pop(0)
            self._slot_owner[slot] = ns
            self._delivery_busy = ns
            res.success = True
            res.message = str(slot)
            self.get_logger().info(
                f'[manager] {ns} -> delivery slot {slot} '
                f'(table now held by {ns})')
            return res

    def _claim_park(self, ns, res):
        with self._lock:
            if not self._free_parks:
                res.success = False
                res.message = 'no free parking vertex'
                return res
            v = self._free_parks.pop(0)
            self._park_owner[v] = ns
            res.success = True
            res.message = str(v)
            self.get_logger().info(f'[manager] {ns} -> parking vertex {v}')
            return res

    # --- watching the fleet ---------------------------------------------------
    def _on_status(self, ns, msg):
        try:
            _who, state, detail = msg.data.split('|', 2)
        except ValueError:
            return
        self.status[ns] = (state, detail)
        # REMEMBER THAT THE RACK GOT THERE, separately from how the errand
        # ended. A robot that places its rack and then fails on the drive to
        # its parking vertex HAS delivered; reporting it as "did not deliver"
        # because its LAST status is 'failed' describes the wrong robot. That
        # is exactly what the first version of the summary below did -- it
        # credited r3, whose rack ended up on the floor, and blamed r1, whose
        # rack was sitting on the table where it belonged.
        if state == 'delivered':
            self.delivered.add(ns)
        # Release the delivery table the moment the rack is down and the robot
        # has backed off -- not when it finishes parking, which would serialise
        # the whole fleet behind one robot's drive across the lobby.
        if state == 'holding':
            with self._lock:
                self.holding.add(ns)
        # THE TURN ENDS AT 'done' OR 'failed' -- see _claim_slot. 'delivered'
        # only means the rack is down; the robot is still at the standoff.
        if state in ('done', 'failed'):
            with self._lock:
                self.finished.add(ns)
                self.holding.discard(ns)
                if self._delivery_busy == ns:
                    self._delivery_busy = None
                    self.get_logger().info(f'[manager] {ns} released the delivery table')

    # --- dispatch -------------------------------------------------------------
    def wait_ready(self, ns, timeout_sec=600.0):
        """Is this robot localized, and STAYING localized?

        A bare "does map -> <ns>/base_link resolve" check is not enough, and
        that is not a theoretical worry. nav_bringup RETRIES a stalled Nav2
        bring-up by calling RESET and starting again, so during a retry storm a
        robot's navigate_to_pose action and its transforms APPEAR AND DISAPPEAR.
        A launch gate that sampled once caught one of those windows and released
        the whole fleet while the third robot was mid-retry -- it was then given
        an errand it could not possibly run, and reported failure within
        seconds.

        So this wants the transform to hold for STABLE_FOR consecutive samples
        before calling the robot ready. Cheap, and it turns a coin flip into a
        condition.
        """
        STABLE_FOR = 6              # 3 s of continuous localization
        deadline = time.time() + timeout_sec
        good = 0
        while time.time() < deadline and rclpy.ok():
            try:
                self.tf_buffer.lookup_transform('map', f'{ns}/base_link',
                                                rclpy.time.Time())
                good += 1
                if good >= STABLE_FOR:
                    return True
            except Exception:
                if good:
                    self.get_logger().info(
                        f'[manager] {ns} localization flickered -- still coming up')
                good = 0
            time.sleep(0.5)
        return False

    def dispatch(self):
        """Give every robot an errand.

        Departures are separated by the depart_stagger parameter; the
        departure ORDER below still matters for the few seconds they are all in
        the formation together.
        """
        pairs = list(zip(self.robots, self.collections))
        for ns, (name, colour, table, rack, stand) in pairs:
            self.assignment[ns] = (name, colour)

        # LEAVE SOUTHERNMOST FIRST, because every route out of the lobby goes
        # south past the reception counter and the formation's apex sits at the
        # NORTH vertex. Dispatched in list order, that apex robot has to drive
        # BETWEEN the two front-rank robots while they are still parked -- and a
        # parked robot is a 0.99 x 0.67 m obstacle, not a gap. Measured: r1 left
        # first, wedged itself against a stationary r2 within three metres, spun
        # its wheels, and its AMCL drifted 3.28 m -- far enough that it believed
        # it was inside the reception desk, which is a lethal cell, so its
        # planner then refused every goal including ones a metre away.
        #
        # Sorting by y means the robots nearest the exit go first and the apex
        # goes last, by which time the lobby ahead of it is empty. It costs
        # nothing: the stagger was going to serialise them anyway.
        pos = {ns: (x, y) for ns, x, y, _ in ROBOTS}
        pairs.sort(key=lambda pr: pos[pr[0]][1])
        self.get_logger().info(
            '[manager] departure order (southernmost first): '
            + ', '.join(p[0] for p in pairs))
        self.get_logger().info('[manager] assignment: ' + ', '.join(
            f'{ns}->{c} at {n}' for ns, (n, c) in self.assignment.items()))

        # EVERY ROBOT IS CONFIRMED READY BEFORE ANY ROBOT IS SENT.
        #
        # wait_ready blocks until that robot's localization holds steady, and it
        # used to be called inside the dispatch loop -- which meant a stack that
        # took an extra thirty seconds to settle held up the errands of robots
        # that had been sitting ready the whole time. With the stagger at zero
        # that would have quietly reintroduced exactly the serialisation the
        # zero is meant to remove, just keyed on bring-up time instead of a
        # timer. Checking everyone first, then publishing, makes "at the same
        # time" mean it.
        ready = []
        for ns, spec in pairs:
            if self.wait_ready(ns):
                self.get_logger().info(f'[manager] {ns} is ready')
                ready.append((ns, spec))
            else:
                self.get_logger().error(
                    f'[manager] {ns} never became localized -- no errand for it')
                self.status[ns] = ('failed', 'never localized')

        for i, (ns, (name, colour, table, rack, stand)) in enumerate(ready):
            if i and self.depart_stagger > 0.0:
                self.get_logger().info(
                    f'[manager] holding {ns} for {self.depart_stagger:.0f}s so '
                    f'{ready[i-1][0]} can clear the formation')
                time.sleep(self.depart_stagger)
            msg = String()
            msg.data = '|'.join([
                colour,
                ','.join(f'{v:.6f}' for v in table),
                ','.join(f'{v:.6f}' for v in rack),
                ','.join(f'{v:.6f}' for v in stand)])
            self.task_pubs[ns].publish(msg)
            self.get_logger().info(f'[manager] dispatched {ns} -> {colour} at {name}')

    def all_done(self):
        return all(self.status[ns][0] in ('parked', 'done', 'failed')
                   for ns in self.robots)

    # --- what RViz shows ------------------------------------------------------
    def _publish_board(self):
        lines = [f'{ns}: {self.status[ns][0]} {self.status[ns][1]}'.rstrip()
                 for ns in self.robots]
        filled = ''.join('X' if i in self._slot_owner else '.'
                         for i in range(len(self.slots)))
        lines.append(f'delivery row [{filled}]  table: '
                     f'{self._delivery_busy or "free"}')
        self.board_pub.publish(String(data='\n'.join(lines)))

    def _marker(self, mid, mtype, pose, scale, rgba, ns='fleet', text=''):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.position.x = float(pose[0])
        m.pose.position.y = float(pose[1])
        m.pose.position.z = float(pose[2])
        yaw = pose[3] if len(pose) > 3 else 0.0
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        # float() ON EVERYTHING, because these are ROS float32 fields and
        # rclpy asserts rather than converting. A single `1` instead of `1.0`
        # anywhere in a colour tuple raises inside the timer callback, which
        # kills the executor thread -- the node stays alive, the services stop
        # being served, and the first robot to ask for a delivery slot hangs
        # until its own timeout. Cheap to prevent, expensive to diagnose.
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
        m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in rgba)

        m.text = text
        return m

    def _publish_markers(self):
        """Draw the job itself: what is where, who owns what, and what is done.

        The robots, the map and the plans are RViz's own displays; this adds the
        things that exist only in this node's head -- which slot is taken, which
        parking vertex is claimed, and where the errands are.
        """
        arr = MarkerArray()
        mid = 0

        # collection tables, coloured by the rack they carry
        for name, colour, table, rack, stand in self.collections:
            rgba = COLOUR_RGBA[colour]
            arr.markers.append(self._marker(
                mid, Marker.CUBE, (table[0], table[1], 0.16, table[2]),
                (RT.TABLE_LONG, RT.TABLE_SHORT, 0.32), (*rgba[:3], 0.25)))
            mid += 1
            arr.markers.append(self._marker(
                mid, Marker.TEXT_VIEW_FACING, (table[0], table[1], 1.0),
                (0.0, 0.0, 0.45), (1, 1, 1, 1),
                text=f'{name}\n{colour}'))
            mid += 1
            # the standoff the robot is sent to
            arr.markers.append(self._marker(
                mid, Marker.ARROW, (stand[0], stand[1], 0.05, stand[2]),
                (0.8, 0.12, 0.12), (*rgba[:3], 0.9)))
            mid += 1

        # delivery table and its row of slots
        dx, dy, dyaw = RT.delivery_table_pose()
        arr.markers.append(self._marker(
            mid, Marker.CUBE, (dx, dy, 0.16, dyaw),
            (RT.TABLE_LONG, RT.TABLE_SHORT, 0.32), (0.85, 0.85, 0.9, 0.30)))
        mid += 1
        arr.markers.append(self._marker(
            mid, Marker.TEXT_VIEW_FACING, (dx, dy, 1.0), (0, 0, 0.45),
            (1, 1, 1, 1), text='delivery'))
        mid += 1
        for i, slot, stand in self.slots:
            owner = self._slot_owner.get(i)
            rgba = (0.2, 0.9, 0.2, 0.9) if owner else (0.6, 0.6, 0.6, 0.5)
            arr.markers.append(self._marker(
                mid, Marker.CYLINDER, (slot[0], slot[1], RT.TABLE_TOP + 0.02),
                (0.14, 0.14, 0.04), rgba))
            mid += 1
            arr.markers.append(self._marker(
                mid, Marker.TEXT_VIEW_FACING, (slot[0], slot[1], RT.TABLE_TOP + 0.30),
                (0, 0, 0.16), (1, 1, 1, 1),
                text=f'{i}:{owner}' if owner else f'{i}:free'))
            mid += 1

        # parking triangle
        for i, (px, py, pyaw) in enumerate(self.parks):
            owner = self._park_owner.get(i)
            rgba = (0.95, 0.75, 0.1, 0.9) if owner else (0.5, 0.5, 0.5, 0.4)
            arr.markers.append(self._marker(
                mid, Marker.ARROW, (px, py, 0.05, pyaw), (0.7, 0.12, 0.12), rgba))
            mid += 1
            arr.markers.append(self._marker(
                mid, Marker.TEXT_VIEW_FACING, (px, py, 0.6), (0, 0, 0.30),
                (1, 1, 1, 1), text=f'P{i}:{owner}' if owner else f'P{i}'))
            mid += 1

        # a status board floating over the lobby, so the run can be followed
        # without a terminal
        board = '\n'.join(f'{ns}: {self.status[ns][0]} {self.status[ns][1]}'.rstrip()
                          for ns in self.robots)
        arr.markers.append(self._marker(
            mid, Marker.TEXT_VIEW_FACING, (0.0, 8.75, 2.6), (0, 0, 0.40),
            (1, 1, 0.6, 1), text=board))
        mid += 1
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = TaskManager()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        # NO FIXED SLEEP HERE. It used to wait 8 s for the mission nodes to
        # construct and subscribe; the task topic is latched (transient local),
        # so a node that subscribes later still receives its errand, and
        # wait_ready() below already refuses to dispatch to a robot that is not
        # localised. A constant that guards nothing is just delay.
        node.dispatch()
        while rclpy.ok() and not node.all_done():
            time.sleep(2.0)
        undelivered = [ns for ns in node.robots if ns not in node.delivered]
        node.get_logger().info(
            f'[manager] every robot has finished: {len(node.delivered)}/'
            f'{len(node.robots)} racks delivered')
        for ns in node.robots:
            state, detail = node.status[ns]
            note = '' if ns in node.delivered else '   (rack NOT delivered)'
            node.get_logger().info(f'    {ns}: {state} {detail}{note}')
        if undelivered:
            node.get_logger().error(
                f'[manager] MISSION INCOMPLETE -- no rack from '
                f'{", ".join(undelivered)}')
        # Stay alive so the markers and the status board keep being published.
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
