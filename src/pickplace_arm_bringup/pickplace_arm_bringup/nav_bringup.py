"""Bring one robot's Nav2 stack up, and RETRY when the bring-up stalls.

WHY THIS EXISTS
---------------
nav2_lifecycle_manager with autostart makes exactly one attempt and does not
retry. It calls change_state on each managed node in turn, and every one of
those calls is bounded by a timeout that Humble HARDCODES at 2 s -
LifecycleServiceClient::get_state's default - with no parameter to raise it.
The live parameter list on a running manager is only

    attempt_respawn_reconnection, autostart, bond_respawn_max_duration,
    bond_timeout, node_names

so there is genuinely no knob. If one node answers slowly the manager stops
where it stands, logs nothing further, and the robot is left with no
navigate_to_pose server for the rest of the run.

That failure is a coin flip, and the coin is weighted by whatever else the
machine is doing. Measured on 2026-08-17 across three runs of the same launch:

    run 1   r1 stopped at "Configuring controller_server"   r2 came up
    run 2   r1 came up in 3 s                               r2 stopped at
                                                            "Configuring
                                                            behavior_server"
    run 3   r1 stopped at "Configuring amcl"                r2 came up

Different node each time, different robot each time. In every case the node the
manager gave up on went on to reach `inactive` BY ITSELF moments later - the
node was healthy and the manager had simply stopped waiting.

Delays were tried at both levels and are the wrong shape. A fixed settle before
the manager is a constant that is wrong on any other machine, and a per-robot
stagger only decides which robot loses. Waiting for the change_state services to
appear does not work either: a LifecycleNode advertises them in its constructor,
so they exist about a second in, long before the node can answer promptly.

WHAT THIS DOES INSTEAD
----------------------
Runs the manager with autostart FALSE and drives it from here, so a stalled
bring-up is a retryable event rather than a dead robot:

    call STARTUP -> poll every managed node's state -> all active? done.
    otherwise call RESET, which returns the whole set to unconfigured, and
    start over.

RESET before retrying matters: after a partial bring-up some nodes are already
inactive, and CONFIGURE is not a legal transition from there, so a bare second
STARTUP would fail on exactly the nodes that had succeeded. Resetting first
makes every attempt identical to the first one.

Exits 0 even when every attempt fails, with the state of each node logged. A
supervisor that can brick a launch is worse than the failure it is guarding
against, which is the same rule wait_for.py follows.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes


ACTIVE = 3          # lifecycle_msgs/State PRIMARY_STATE_ACTIVE


class NavBringup(Node):
    def __init__(self, args):
        super().__init__('nav_bringup')
        self.args = args
        self.ns = args.namespace.strip('/')
        self.manage = self.create_client(
            ManageLifecycleNodes,
            f'/{self.ns}/lifecycle_manager_navigation/manage_nodes')
        self.state_clients = {
            n: self.create_client(GetState, f'/{self.ns}/{n}/get_state')
            for n in args.node
        }

    def _call(self, client, request, timeout):
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def command(self, value, label):
        req = ManageLifecycleNodes.Request()
        req.command = value
        # Generous, because the whole point is that this call is slow under
        # load. The manager's own internal per-node deadline still applies; this
        # only bounds how long WE wait to hear how it went.
        res = self._call(self.manage, req, self.args.command_timeout)
        ok = bool(res and res.success)
        self.get_logger().info(f'[{self.ns}] {label} -> '
                               f'{"ok" if ok else "reported failure"}')
        return ok

    def states(self):
        """Current lifecycle state id of every managed node, None if unknown."""
        out = {}
        for name, cli in self.state_clients.items():
            res = self._call(cli, GetState.Request(), 5.0)
            out[name] = res.current_state.id if res else None
        return out

    def all_active(self, deadline):
        """Poll until every managed node is ACTIVE, or the deadline passes."""
        while time.time() < deadline:
            st = self.states()
            if all(v == ACTIVE for v in st.values()):
                return True, st
            time.sleep(2.0)
        return False, self.states()

    def run(self):
        log = self.get_logger()
        if not self.manage.wait_for_service(timeout_sec=self.args.wait_manager):
            log.error(f'[{self.ns}] manage_nodes never appeared - '
                      f'is the lifecycle manager running?')
            return False

        for attempt in range(1, self.args.attempts + 1):
            log.info(f'[{self.ns}] bring-up attempt {attempt}'
                     f'/{self.args.attempts}')
            self.command(ManageLifecycleNodes.Request.STARTUP, 'STARTUP')
            ok, st = self.all_active(time.time() + self.args.settle)
            if ok:
                log.info(f'[{self.ns}] all {len(st)} nodes active after '
                         f'{attempt} attempt(s)')
                return True

            stalled = [n for n, v in st.items() if v != ACTIVE]
            log.warn(f'[{self.ns}] attempt {attempt} left {len(stalled)} node(s) '
                     f'not active: {", ".join(sorted(stalled))}')
            if attempt < self.args.attempts:
                # Back to unconfigured, so the retry is a clean first attempt
                # rather than a CONFIGURE aimed at already-configured nodes.
                self.command(ManageLifecycleNodes.Request.RESET, 'RESET')
                time.sleep(self.args.backoff)

        log.error(f'[{self.ns}] gave up after {self.args.attempts} attempts; '
                  f'final states: {self.states()}')
        return False


def main(argv=None):
    argv = remove_ros_args(sys.argv if argv is None else argv)[1:]
    p = argparse.ArgumentParser()
    p.add_argument('--namespace', required=True)
    p.add_argument('--node', action='append', default=[],
                   help='managed node name, repeatable')
    p.add_argument('--attempts', type=int, default=4)
    p.add_argument('--settle', type=float, default=45.0,
                   help='seconds to wait for all nodes to reach active')
    p.add_argument('--backoff', type=float, default=5.0)
    p.add_argument('--command-timeout', type=float, default=120.0)
    p.add_argument('--wait-manager', type=float, default=120.0)
    args, _unknown = p.parse_known_args(argv)

    rclpy.init()
    node = NavBringup(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # Always 0: see the module docstring.
    return 0


if __name__ == '__main__':
    sys.exit(main())
