#!/usr/bin/env python3
"""The two-robot handshake: one latched topic per role, states that only advance.

STAGE 5. Everything up to here has been one robot at a time - the fleet gives
each robot its own sensors, TF island and Nav2 stack precisely so they do not
interfere - and this is the one place two of them have to agree about something.

WHY A TOPIC AND NOT AN ACTION OR A SERVICE. A service call needs the callee up
and serving before the caller asks, and these two nodes start together behind
independent readiness gates that finish minutes apart. A latched topic has no
such ordering requirement: the carrier can announce `collected` while the
receiver's node is still constructing, and the receiver still sees it the
instant it subscribes. That is not a nicety here, it is the normal case - the
carrier's collect leg is a ~45 m drive and a full pick.

THE TOPICS ARE ABSOLUTE, AND THEY HAVE TO BE. Both mission nodes run inside
their robot's namespace, where a relative `handover/carrier` would resolve to
/r1/handover/carrier and /r2/handover/carrier - two private topics that never
meet, and a handshake that silently never completes. These are the one thing in
the whole fleet that is deliberately shared, alongside /map.

STATES ARE AN ORDERED LIST AND A WAIT IS ">=", NOT "==". The latch keeps depth
1, so a peer that publishes `at_handover` and then `released` while nobody is
listening delivers only `released` to a late subscriber. Waiting for equality
would hang forever on a state that has already been superseded, and every such
hang would look exactly like a robot that never got there. Comparing positions
in the list makes a missed intermediate state harmless, which is the property
the latch's depth of 1 actually gives us.

FAILURE IS A STATE, NOT A TIMEOUT. A peer that gives up says so, and the other
robot stops waiting immediately instead of burning its full rendezvous timeout
on a partner that is already parked. Timeouts still exist for the case the
protocol cannot cover - a peer that died without saying anything - and they are
deliberately long, because the legs they cover really do take minutes.
"""
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

CARRIER = 'carrier'
RECEIVER = 'receiver'

# The protocol, in order. A robot only ever moves forward through its own list.
#
#   carrier                          receiver
#   -------                          --------
#   start        node is alive       start        node is alive
#   collected    rack is on board    at_handover  parked at the meet pose
#   at_handover  parked, facing it   holding      rack is on board
#   released     rack is on the floor delivered   rack is on the dock
#   clear        backed out of reach  parked      back in the ring
#   parked       back in the ring
STATES = {
    CARRIER: ['start', 'collected', 'at_handover', 'released', 'clear', 'parked'],
    RECEIVER: ['start', 'at_handover', 'holding', 'delivered', 'parked'],
}
FAILED = 'failed'

PEER = {CARRIER: RECEIVER, RECEIVER: CARRIER}


def topic(role):
    return f'/handover/{role}'


# Latched, so a state published before the peer exists is still delivered to it.
# TRANSIENT_LOCAL is the whole point; the depth of 1 is what makes the ">=" rule
# above necessary rather than merely robust.
LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class HandoverPeer:
    """Mixin: gives a mission node its own state topic and a view of its peer's.

    Mixed in FIRST, so that `super().__init__()` runs the whole mission class
    chain before this sets its endpoints up - creating them needs the Node to
    exist, and the mission chain is what constructs it.

    THE HANDSHAKE GETS ITS OWN NODE, ITS OWN EXECUTOR AND ITS OWN THREAD, AND
    THAT IS NOT TIDINESS - IT IS THE FIX FOR AN OBSERVED FAILURE.

    The first version put the subscription on the mission node, in a callback
    group on the mission node's MultiThreadedExecutor, which is the obvious
    thing to do and works fine in a test with nothing else running. On a real
    run it lost a state change outright: the carrier published `failed`, and
    36 seconds later the receiver's own log still said "peer at collected". Its
    callback had last fired 160 seconds earlier.

    NOT A DDS PROBLEM, and that was worth establishing before changing
    anything. With the run stopped in that state, an outside `ros2 topic echo
    /handover/carrier` returned `data: failed` immediately; `ros2 topic info -v`
    showed one publisher and one subscriber with matching RELIABLE /
    TRANSIENT_LOCAL QoS. The sample had been delivered to the process. It was
    the process that never looked at it.

    What the mission node's executor is busy with is the reason. It carries two
    RGB-D point-cloud subscriptions - 640x480 organised clouds, about 5 MB each,
    at roughly 8 Hz apiece - and rclpy deserialises every one of those into
    Python. On top of that the mission thread calls
    rclpy.spin_until_future_complete on the SAME node for costmap clears and
    parameter sets, which stands up a second executor over the node's wait set
    while the first one is still using it. That combination is already known
    here to be unstable: _run in mission_2.py catches the RCLError it throws
    ("wait set index out of bounds") and resumes rather than dying.

    A one-line String that gates a two-robot rendezvous has no business queued
    behind any of that. On its own node it shares nothing: its own wait set, its
    own executor, its own thread, no point clouds, and nothing else ever calls
    spin on it. The publisher moves too - publishing is immediate and was never
    the problem, but splitting the pair across two nodes for no reason is how
    the next person ends up debugging this twice.
    """

    # Set by the subclass to CARRIER or RECEIVER.
    ROLE = None

    def __init__(self):
        super().__init__()
        self._peer_role = PEER[self.ROLE]
        self._peer_state = None
        self._own_state = None

        # Same namespace as the mission node, so `ros2 node list` groups them
        # together. The topics themselves are absolute and unaffected.
        ns = self.get_namespace()
        self._hs_node = rclpy.create_node(f'handover_{self.ROLE}', namespace=ns)
        self._state_pub = self._hs_node.create_publisher(
            String, topic(self.ROLE), LATCHED)
        self._hs_node.create_subscription(
            String, topic(self._peer_role), self._peer_cb, LATCHED)
        self._hs_exec = SingleThreadedExecutor()
        self._hs_exec.add_node(self._hs_node)
        self._hs_thread = threading.Thread(
            target=self._spin_handshake, name=f'handover_{self.ROLE}', daemon=True)
        self._hs_thread.start()

        self.get_logger().info(
            f'[handover] {self.ROLE}: announcing on {topic(self.ROLE)}, '
            f'watching {topic(self._peer_role)} (own node {self._hs_node.get_name()})')

    def _spin_handshake(self):
        """Service the handshake node forever, and never die of one bad spin.

        Same defensive shape as _run's executor loop in mission_2.py: if this
        thread throws and exits, nothing says so and both robots simply wait out
        their timeouts - the failure this whole class exists to avoid.

        SHUTDOWN IS A NORMAL EXIT, AND IT HAS TO BE REACHABLE. A bare spin()
        blocks until the context dies, so this thread is still inside it when
        rclpy.shutdown() runs and the process tears down around it - which
        aborts with "terminate called without an active exception" and dumps
        core on an otherwise clean, successful run. Measured: that is exactly
        what a bare spin() did here. A timed spin_once returns to the top of the
        loop several times a second, so rclpy.ok() going False ends the thread
        by itself, before anything is destroyed underneath it.

        0.2 s is the worst-case shutdown lag and costs one wakeup per 200 ms on
        a subscription that carries one short string per leg.
        """
        try:
            while rclpy.ok():
                try:
                    self._hs_exec.spin_once(timeout_sec=0.2)
                except ExternalShutdownException:
                    return
                except Exception as exc:
                    if not rclpy.ok():
                        return
                    self.get_logger().warn(
                        f'[handover] spin error, resuming: {exc}')
                    time.sleep(0.1)
        finally:
            try:
                self._hs_exec.shutdown()
                self._hs_node.destroy_node()
            except Exception:
                pass

    def _peer_cb(self, msg):
        if msg.data != self._peer_state:
            self.get_logger().info(f'[handover] {self._peer_role} -> {msg.data}')
        self._peer_state = msg.data

    # --- announcing ---------------------------------------------------------
    def announce(self, state):
        """Publish this robot's new state. Unknown states are a programming
        error and are rejected loudly rather than published, because a state
        outside the list can never satisfy a peer's ">=" wait and would hang the
        other robot for its full timeout with nothing in the log to explain it."""
        if state != FAILED and state not in STATES[self.ROLE]:
            self.get_logger().error(
                f'[handover] refusing to announce unknown state {state!r}')
            return
        self._own_state = state
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)
        self.get_logger().info(f'[handover] {self.ROLE} -> {state}')

    def announce_failure(self, why):
        self.get_logger().error(f'[handover] {self.ROLE} FAILED: {why}')
        self.announce(FAILED)

    # --- waiting ------------------------------------------------------------
    def peer_reached(self, state):
        """Has the peer reached `state` or gone past it? None while it has said
        nothing at all, False on a state before it, True at or after it."""
        seen = self._peer_state
        if seen is None:
            return None
        if seen == FAILED:
            return False
        order = STATES[self._peer_role]
        if seen not in order:
            return False
        return order.index(seen) >= order.index(state)

    def wait_for_peer(self, state, timeout_sec, poll_sec=0.5):
        """Block until the peer is at `state` or past it. Returns False if it
        announced a failure or if the timeout expires with nothing heard.

        This runs on the mission thread while the executor spins the node in
        another, which is what lets a blocking wait here still receive the
        message it is waiting for - the same arrangement every rclpy
        spin_until_future_complete call in this package relies on.
        """
        log = self.get_logger()
        log.info(f'[handover] waiting for {self._peer_role} to reach {state} '
                 f'(up to {timeout_sec:.0f} s)')
        deadline = time.time() + timeout_sec
        announced = 0.0
        while time.time() < deadline:
            if self._peer_state == FAILED:
                log.error(f'[handover] {self._peer_role} reported FAILED while '
                          f'we waited for {state} -- giving up')
                return False
            if self.peer_reached(state):
                log.info(f'[handover] {self._peer_role} is at {self._peer_state} '
                         f'(>= {state})')
                return True
            # A silent wait of several minutes is indistinguishable from a hung
            # node in the log, and these waits legitimately last that long.
            if time.time() - announced > 30.0:
                announced = time.time()
                left = deadline - time.time()
                log.info(f'[handover] still waiting for {self._peer_role} '
                         f'{state} (peer at {self._peer_state}, {left:.0f} s left)')
            time.sleep(poll_sec)
        log.error(f'[handover] timed out waiting for {self._peer_role} to reach '
                  f'{state} (last seen: {self._peer_state})')
        return False
