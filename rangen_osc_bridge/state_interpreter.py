#!/usr/bin/env python3
"""
state_interpreter — arm-posture scalars derived from TF, for the OSC bridge.

Where ee_osc_bridge turns one pose stream into velocity and acceleration, this
node reads the whole arm's TF tree and reduces it to the few scalars the
composer asked for by name.  They are published as ordinary ROS topics:

  /rangen/state/elbow_state    std_msgs/Float32   elbow z − EE z          (m)
  /rangen/state/reach          std_msgs/Float32   |EE − arm base|         (m)
  /rangen/state/reach_discrete std_msgs/Int32     0 near / 1 mid / 2 far

and ee_osc_bridge forwards them to Max as

  /rangen/elbow_state          1 arg
  /rangen/reach                1 arg
  /rangen/reach/discrete       1 arg

via the `generic:` rows in config/osc_signals.yaml.  This node deliberately
never opens a UDP socket itself: keeping every OSC address, its arity and its
send rate in the bridge means the composer's contract lives in exactly one
place, and these three ride the same 50 Hz tick as the EE block instead of
arriving on their own schedule.

Why TF rather than /ee_pose_actual
----------------------------------
The elbow has no pose topic, so TF is required for it either way — and
elbow_state is a *difference* of two z values.  Taking both ends from the same
TF snapshot means the number can never be the difference between two poses
sampled from different sources at different instants.  reach comes from the same
lookup for the same reason.  The frames match rangen_kinova_cpp's
kortex_cartesian_position_pid.hpp (elbow gen3_forearm_link, EE
gen3_robotiq_85_tool_link, control frame gen3_base_link).

Nothing is published until TF actually resolves: a missing transform emits
silence, not a zero, matching the generic layer's rule that zero would tell Max
a dead sensor reads exactly 0.0.
"""

import math

import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


# Discrete reach levels.  These are the values that go on the wire, so they are
# named once here rather than spelled as bare ints at each use.
REACH_NEAR = 0
REACH_MID = 1
REACH_FAR = 2


def reach_level(reach: float, near_m: float, far_m: float,
                prev=None, hysteresis: float = 0.0) -> int:
    """0 below `near_m`, 2 above `far_m`, 1 in between.

    The plain thresholds are exact: reach == near_m is level 1 ("below 0.6"
    means strictly below), reach == far_m is level 1 likewise.

    `hysteresis` > 0 widens whichever band `prev` is currently in by that much,
    so a hand parked on a threshold doesn't chatter 0/1/0/1 at the send rate and
    retrigger something in Max 50 times a second.  It defaults to 0, i.e. the
    thresholds behave exactly as specified.
    """
    lo, hi = near_m, far_m
    h = max(0.0, hysteresis)
    if h and prev is not None:
        if prev == REACH_NEAR:
            lo += h             # must reach further than near_m+h to leave 'near'
        elif prev == REACH_FAR:
            hi -= h             # must come closer than far_m-h to leave 'far'
        else:
            lo -= h             # must overshoot either edge to leave the middle
            hi += h
    if reach < lo:
        return REACH_NEAR
    if reach > hi:
        return REACH_FAR
    return REACH_MID


class StateInterpreter(Node):

    def __init__(self):
        super().__init__('state_interpreter')

        # ── frames ───────────────────────────────────────────────────────────
        # Defaults are the Gen3 frame names used by rangen_kinova_cpp.
        self.declare_parameter('base_frame', 'gen3_base_link')
        self.declare_parameter('elbow_frame', 'gen3_forearm_link')
        self.declare_parameter('ee_frame', 'gen3_robotiq_85_tool_link')

        # Matches ee_osc_bridge's send_rate_hz: the bridge holds the last value
        # between updates, so publishing slower than it sends would just make
        # the OSC stream stair-step.
        self.declare_parameter('publish_rate_hz', 50.0)
        # How long to wait for a transform.  Short: this runs on a timer, and a
        # late transform is worth less than the next one arriving on time.
        self.declare_parameter('tf_timeout_sec', 0.05)

        # ── reach thresholds ─────────────────────────────────────────────────
        self.declare_parameter('reach_near_m', 0.6)
        self.declare_parameter('reach_far_m', 0.95)
        self.declare_parameter('reach_hysteresis_m', 0.0)

        # ── output topics ────────────────────────────────────────────────────
        # The osc_signals.yaml generic rows name these, so renaming one here
        # means editing that file too.
        self.declare_parameter('topic_prefix', '/rangen/state')

        self._base_frame = self.get_parameter('base_frame').value
        self._elbow_frame = self.get_parameter('elbow_frame').value
        self._ee_frame = self.get_parameter('ee_frame').value
        self._tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self._near_m = float(self.get_parameter('reach_near_m').value)
        self._far_m = float(self.get_parameter('reach_far_m').value)
        self._hysteresis = float(self.get_parameter('reach_hysteresis_m').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        if self._far_m < self._near_m:
            self.get_logger().warn(
                f'reach_far_m ({self._far_m}) is below reach_near_m ({self._near_m}) — '
                f'level 1 can never be reached')

        prefix = str(self.get_parameter('topic_prefix').value).rstrip('/')
        self._pub_elbow = self.create_publisher(Float32, f'{prefix}/elbow_state', 10)
        self._pub_reach = self.create_publisher(Float32, f'{prefix}/reach', 10)
        self._pub_discrete = self.create_publisher(Int32, f'{prefix}/reach_discrete', 10)

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lst = tf2_ros.TransformListener(self._tf_buf, self)

        self._level = None          # last emitted discrete level, for hysteresis
        self._publish_count = 0
        self._log_every = max(1, int(rate * 2.0))       # log every ~2 s

        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'state_interpreter started\n'
            f'  base frame  : {self._base_frame}\n'
            f'  elbow frame : {self._elbow_frame}\n'
            f'  ee frame    : {self._ee_frame}\n'
            f'  publishing  : {prefix}/elbow_state, {prefix}/reach, '
            f'{prefix}/reach_discrete  at {rate:.0f} Hz\n'
            f'  reach levels: 0 below {self._near_m:.2f} m, 1 between, '
            f'2 above {self._far_m:.2f} m'
            + (f'  (±{self._hysteresis * 100:.1f} cm hysteresis)'
               if self._hysteresis > 0 else '')
            + '\n'
            f'  OSC         : forwarded by ee_osc_bridge as /rangen/elbow_state, '
            f'/rangen/reach, /rangen/reach/discrete\n'
            f'                (the generic: rows in config/osc_signals.yaml)'
        )

    def _translation(self, child_frame):
        """(x, y, z) of `child_frame` in base_frame, or None if TF can't say yet.

        rclpy.time.Time() asks for the latest available transform rather than
        one stamped now, so a TF publisher running a few ms behind wall clock
        still answers instead of raising ExtrapolationException every tick.
        """
        try:
            tf = self._tf_buf.lookup_transform(
                self._base_frame, child_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=self._tf_timeout))
        except tf2_ros.TransformException:
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def _tick(self):
        ee = self._translation(self._ee_frame)
        elbow = self._translation(self._elbow_frame)
        if ee is None or elbow is None:
            missing = ', '.join(
                f'{self._base_frame} → {f}'
                for f, p in ((self._ee_frame, ee), (self._elbow_frame, elbow))
                if p is None)
            # Throttled, not once: TF dropping out mid-take is worth seeing
            # again, but not 50 times a second.
            self.get_logger().warn(f'no transform for {missing} — nothing published',
                                   throttle_duration_sec=5.0)
            return

        # elbow z minus EE z: positive when the elbow rides above the hand,
        # negative when the arm reaches down over the wrist.
        elbow_state = elbow[2] - ee[2]

        # Straight-line distance from the arm base, which is the origin of
        # base_frame — so the norm of the EE translation is that distance.
        reach = math.sqrt(ee[0] * ee[0] + ee[1] * ee[1] + ee[2] * ee[2])

        level = reach_level(reach, self._near_m, self._far_m,
                            self._level, self._hysteresis)
        if level != self._level and self._level is not None:
            self.get_logger().info(
                f'reach level {self._level} → {level}  ({reach:.3f} m)')
        self._level = level

        self._pub_elbow.publish(Float32(data=float(elbow_state)))
        self._pub_reach.publish(Float32(data=float(reach)))
        self._pub_discrete.publish(Int32(data=int(level)))

        self._publish_count += 1
        if self._publish_count % self._log_every == 0:
            self.get_logger().info(
                f'[#{self._publish_count:6d}]  '
                f'elbow_state={elbow_state:+.3f} m  '
                f'reach={reach:.3f} m  level={level}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = StateInterpreter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
