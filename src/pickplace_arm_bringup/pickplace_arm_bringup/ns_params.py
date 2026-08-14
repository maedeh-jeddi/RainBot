"""Rewrite a single-robot Nav2/AMCL parameter file for one namespaced robot.

WHY NOT nav2_common's RewrittenYaml. That helper substitutes by KEY name, and
nav2_params.yaml uses one key, `global_frame`, with two different meanings: the
local costmap plans in `odom` and the global costmap plans in `map`. A key-based
rewrite has to give both the same value and breaks one of them.

So this rewrites by VALUE instead. `odom` becomes `r1/odom` wherever it appears,
`map` is left alone because the map frame is genuinely shared between robots,
and topic names pick up a `/r1` prefix. That distinction - frames get prefixed,
the map frame does not - is the whole point of a multi-robot TF tree: each robot
owns its own odom island, and AMCL is what ties them to the one shared map.
"""

import os
import tempfile

import yaml

# value -> what it becomes for robot <ns>. Frames first, then topics.
FRAME_VALUES = ('base_link', 'base_footprint', 'odom')
TOPIC_VALUES = ('/scan', 'scan', '/imu',
                '/diff_drive_controller/odom',
                '/diff_drive_controller/cmd_vel_unstamped')


def _rewrite(value, ns):
    if not isinstance(value, str):
        return value
    if value in FRAME_VALUES:
        return f'{ns}/{value}'
    if value in TOPIC_VALUES:
        return f'/{ns}{value}' if value.startswith('/') else f'/{ns}/{value}'
    return value


def _walk(node, ns):
    if isinstance(node, dict):
        out = {k: _walk(v, ns) for k, v in node.items()}
        # THE STATIC LAYER NEEDS ITS MAP TOPIC SPELLED OUT.
        #
        # Every other node in the stack picks the shared map up from a launch
        # remapping of `map` -> `/map`. The costmaps cannot: Costmap2DROS builds
        # its internal node with a fresh argument list carrying only its own
        # namespace, so the parent process's remap rules never reach it, and the
        # static layer quietly subscribes to /r1/map where nobody publishes. The
        # costmap then never resizes off its default extent and the planner
        # rejects every goal with "Robot is out of bounds of the costmap!" while
        # AMCL - a plain node, so remapped correctly - localises perfectly. The
        # two symptoms look unrelated, which is what makes this worth naming.
        if 'static_layer' in out and isinstance(out['static_layer'], dict):
            out['static_layer']['map_topic'] = '/map'
        # A COSTMAP THAT CANNOT FIND THE ROBOT ABORTS THE GOAL IN SILENCE.
        #
        # planner_server asks the costmap where the robot is, and if that lookup
        # fails it terminates the goal WITHOUT LOGGING ANYTHING - the caller sees
        # a bare ABORTED. That is what a mission run looked like from outside:
        # Nav2 refusing a route that, called directly with an explicit start from
        # the robot's own pose, planned successfully every time.
        #
        # The lookup is map -> <ns>/base_link, and it is healthy at rest (20 of
        # 20, 0.02 s old) but throws ExtrapolationException while the fleet is
        # driving. 0.3 s is simply too tight a window on a machine running four
        # robots: AMCL post-dates its own map->odom by a full second, the EKF
        # runs at 22 Hz, and under load the composition falls outside 0.3 s.
        # 1.0 s matches what AMCL already allows itself.
        #
        # Fleet-only, because it is applied by the rewriter: the single-robot
        # missions keep nav2_params.yaml untouched.
        #
        # The key is ADDED, not overwritten - neither costmap sets it, so both
        # were silently running on the 0.3 s default. A section is a costmap
        # when it declares both frames it transforms between; that also keeps
        # this away from the two places that DO set the value on purpose, the
        # pure-pursuit controller (0.2) and the behaviour server (0.1), where a
        # slack window would mean acting on a stale pose rather than failing to
        # find one.
        if 'global_frame' in out and 'robot_base_frame' in out \
                and 'plugins' in out:
            out['transform_tolerance'] = 1.0
        # DRIVE AT THE SPEED THE REST OF THE STACK ALREADY ALLOWS. The limit
        # chain is: the pure pursuit controller asks for desired_linear_vel, the
        # velocity smoother caps it at 0.7, and diff_drive_controller's own
        # limit is 0.8 m/s. nav2_params.yaml asks for 0.55, so the binding limit
        # was the request, not the hardware - the robot was leaving a quarter of
        # its speed unused on routes 40 m long.
        #
        # Every cap in that chain is raised together, or the lowest one wins:
        # 0.85 requested, 1.0 at the smoother, 1.0 at the diff drive controller
        # (see diff_drive_frames). Fleet-only - the single-robot missions keep
        # their tuned 0.55 and the shared 0.8 limit.
        #
        # WORTH KNOWING BEFORE RAISING IT FURTHER: this robot was already
        # mislocalised by 1.44 m at the end of a 40 m run, and wheel odometry
        # error grows with speed on a skid-steer. Faster driving is not free
        # here; it trades against how well AMCL keeps up.
        if 'desired_linear_vel' in out:
            out['desired_linear_vel'] = 0.85
        # The velocity smoother caps whatever the controller asks for, so it has
        # to come up too or 0.85 is clipped straight back to 0.70.
        if 'max_velocity' in out and isinstance(out['max_velocity'], list):
            out['max_velocity'] = [1.0, 0.0, 2.2]
        # AMCL MUST NOT POST-DATE ITS TRANSFORM BY MORE THAN THE REST OF THE
        # CHAIN CAN REACH.
        #
        # amcl_tugbot.yaml sets transform_tolerance 1.0, so AMCL stamps
        # map->odom a full second into the FUTURE. That is normal AMCL practice
        # and harmless on its own. It is not harmless in this chain: the EKF
        # stamps odom->base_link at the present instant, so map->odom's newest
        # sample is ahead of now and odom->base_link's newest is behind it, and
        # tf2 can find no time where both are valid. Measured, standing still:
        #
        #   map     <- r1/odom       15/15 ok, stamp 0.780 s in the future
        #   r1/odom <- r1/base_link  15/15 ok, stamp 0.010 s in the past
        #   map     <- r1/base_link   0/15    ExtrapolationException
        #
        # Both links healthy, the composition always failing. planner_server
        # calls exactly that composition through getRobotPose(), fails, and
        # terminates the goal WITHOUT LOGGING - a bare ABORTED, on a route that
        # plans fine when the start pose is supplied by hand.
        #
        # It bites hardest when the robot is stationary, because AMCL stops
        # updating and its future-stamped transform just sits there. That is why
        # the drive out always worked and every goal after the pick did not.
        #
        # The costmap's own transform_tolerance does NOT fix this - that is a
        # lookup timeout, not a stamp-alignment problem.
        #
        # 0.5 rather than the 0.1 tried first: 0.1 is short enough that AMCL's
        # transform can expire between its own updates while the robot is
        # standing still, and it does NOT affect localisation accuracy either
        # way - this parameter only says how far ahead AMCL stamps, not how well
        # the particle filter tracks.
        if 'base_frame_id' in out and 'odom_frame_id' in out:
            out['transform_tolerance'] = 0.5
        return out
    if isinstance(node, list):
        return [_walk(v, ns) for v in node]
    return _rewrite(node, ns)


def namespaced_params(source, ns, overrides=None):
    """Return a path to `source` rewritten for robot `ns`.

    The whole document is nested under the namespace as well, because a node
    launched into /r1 is called /r1/amcl and a parameter file keyed on plain
    `amcl:` will not match it - the node then comes up with no parameters at
    all, which for AMCL means default frames and silent mislocalisation.
    """
    with open(source) as fh:
        data = yaml.safe_load(fh)

    data = _walk(data, ns)
    for node_name, params in (overrides or {}).items():
        data.setdefault(node_name, {}).setdefault('ros__parameters', {}).update(params)

    out = os.path.join(tempfile.mkdtemp(prefix=f'{ns}_params_'),
                       os.path.basename(source))
    with open(out, 'w') as fh:
        yaml.safe_dump({ns: data}, fh, default_flow_style=False)
    return out


def diff_drive_frames(ns):
    """Write the per-robot frame overlay for diff_drive_controller.

    arm_controllers.yaml is shared by every robot and wildcards its node keys,
    so it cannot say which robot it is describing and ships the unprefixed
    `odom` / `base_link`. The EKF does not mind (it reads body-frame vx, which
    carries no frame), but Nav2's controller_server checks the odometry's frame
    against the tree it is planning in. Generated rather than committed as one
    file per robot, so adding a fifth robot stays a one-line change.
    """
    out = os.path.join(tempfile.mkdtemp(prefix=f'{ns}_ddframes_'),
                       f'diff_drive_frames_{ns}.yaml')
    with open(out, 'w') as fh:
        yaml.safe_dump({f'/{ns}/diff_drive_controller': {'ros__parameters': {
            'odom_frame_id': f'{ns}/odom',
            'base_frame_id': f'{ns}/base_link',
            # The last cap in the chain. arm_controllers.yaml allows 0.8 m/s;
            # raised here so the smoother's 1.0 is not clipped by the controller
            # itself. Fleet-only, like the frames - the single-robot missions
            # keep the shared file's 0.8.
            'linear.x.max_velocity': 1.0}}}, fh)
    return out
