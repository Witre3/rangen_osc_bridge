#!/usr/bin/env python3
"""
ee_osc_bridge — ROS2 node that forwards end-effector kinematics as OSC.

Commanded pose source: /ee_pose_ref (geometry_msgs/PoseStamped, frame gen3_base_link).
Always available when the teleop UI stack is running.

Actual (measured) pose source: /ee_pose_actual (geometry_msgs/PoseStamped,
frame gen3_base_link) — published by haply_delta_ee_pose at 100 Hz.  That node
uses TF FK as its source (gen3_base_link → gen3_robotiq_85_tool_link), with
IGPS odometry as a higher-priority override when available.

Velocity and acceleration are NOT published by any upstream node; they are
derived here via finite differences and smoothed with an EMA filter.

OSC address schema (all per-timer-tick, flat UDP messages — no #bundle framing):

  Commanded EE (source: /ee_pose_ref):
    /rangen/ee/pos               x y z    (m, frame gen3_base_link)
    /rangen/ee/quat              x y z w
    /rangen/ee/vel_lin           x y z    (m/s, EMA)
    /rangen/ee/vel_lin/mag       scalar
    /rangen/ee/vel_lin/mag/norm  0–1, via 'vel_lin_mag' curve in norm_curves.yaml
    /rangen/ee/vel_ang           x y z    (rad/s, EMA)
    /rangen/ee/accel_lin         x y z    (m/s², EMA)
    /rangen/ee/accel_lin/mag     scalar
    /rangen/ee/accel_lin/mag/norm  0–1, via 'accel_lin_mag' curve

  Actual (measured) EE (source: /ee_pose_actual from haply_delta_ee_pose, TF FK):
    /rangen/ee/act/pos               x y z    (m, frame gen3_base_link)
    /rangen/ee/act/vel_lin           x y z    (m/s, EMA)
    /rangen/ee/act/vel_lin/mag       scalar
    /rangen/ee/act/vel_lin/mag/norm  0–1, via 'vel_lin_mag' curve
    /rangen/ee/act/vel_ang           x y z    (rad/s, EMA)
    /rangen/ee/act/accel_lin         x y z    (m/s², EMA)
    /rangen/ee/act/accel_lin/mag     scalar
    /rangen/ee/act/accel_lin/mag/norm  0–1, via 'accel_lin_mag' curve

The /mag/norm outputs go through a tunable breakpoint curve (norm_curve.py)
loaded from norm_curves.yaml and hot-reloaded, so edits made live in
scripts/curve_editor.py take effect without restarting this node.

Generic signals
---------------
Beyond the 17 EE addresses above, any ROS2 topic/field selected in
scripts/signal_editor.py is forwarded too, as /rangen/<topic>/<field> with a
fixed argument count.  Both the EE switches and the generic list live in
config/osc_signals.yaml, which this node polls by mtime and hot-reloads the
same way it does norm_curves.yaml — so the GUI applies changes live, with no
restart and no ROS parameter round-trip.

If osc_signals.yaml is missing, empty or malformed, all 17 EE signals are sent
and no generic ones: exactly the behaviour from before that file existed.
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from .ee_kinematics import EeState, OscSender, emit_builtin, pose_from_msg
from .generic_sources import GenericSourceManager
from .norm_curve import NormCurve, load_curves
from .osc_signals import BUILTIN_ADDRESSES, BUILTIN_SIGNALS, SignalConfig, load_signal_config


def _default_config_path(filename: str) -> str:
    """This package's installed config/<filename>, or '' if not installed."""
    try:
        return os.path.join(
            get_package_share_directory('rangen_osc_bridge'),
            'config', filename,
        )
    except Exception:
        return ''


class EeOscBridge(Node):

    def __init__(self):
        super().__init__('ee_osc_bridge')

        self.declare_parameter('osc_target_ip', '127.0.0.1')
        self.declare_parameter('osc_target_port', 9000)
        # Second target so both the composer and this machine's own
        # osc_visualizer.py can watch the same stream. Defaults to
        # localhost; leave osc_target_ip_2 empty to disable it.
        self.declare_parameter('osc_target_ip_2', '127.0.0.1')
        self.declare_parameter('osc_target_port_2', 9000)
        self.declare_parameter('send_rate_hz', 50.0)
        # EMA alpha: 1.0 = raw, 0.0 = frozen.  ~0.1–0.3 gives musical smoothing.
        self.declare_parameter('smoothing_alpha', 0.15)
        self.declare_parameter('pose_topic', '/ee_pose_ref')
        self.declare_parameter(
            'actual_pose_topic',
            '/ground_truth/odometry_igps/gen3_robotiq_85_tool_link',
        )
        self.declare_parameter('norm_curves_yaml', _default_config_path('norm_curves.yaml'))
        # How often to check the curve file's mtime and hot-reload it, so
        # edits made live in curve_editor.py apply without restarting this
        # node.  Cheap (one stat() call) so a short interval is fine.
        self.declare_parameter('norm_curves_reload_period_sec', 1.0)
        # Signal selection: which of the 17 built-in EE addresses are sent, plus
        # any generic topic/field the user picked in signal_editor.py.  Same
        # mtime hot-reload deal as the curves above.
        self.declare_parameter('osc_signals_yaml', _default_config_path('osc_signals.yaml'))
        self.declare_parameter('osc_signals_reload_period_sec', 1.0)
        # Ceiling on generic signals, so a runaway config can't turn this node
        # into a packet cannon aimed at the composer's laptop.
        self.declare_parameter('generic_max_signals', 32)

        ip     = self.get_parameter('osc_target_ip').value
        port   = self.get_parameter('osc_target_port').value
        ip_2   = self.get_parameter('osc_target_ip_2').value
        port_2 = self.get_parameter('osc_target_port_2').value
        rate = self.get_parameter('send_rate_hz').value
        self._alpha = self.get_parameter('smoothing_alpha').value
        pose_topic        = self.get_parameter('pose_topic').value
        actual_pose_topic = self.get_parameter('actual_pose_topic').value

        self._curves_path = self.get_parameter('norm_curves_yaml').value
        self._curves_reload_period = self.get_parameter('norm_curves_reload_period_sec').value
        self._curves_mtime = None
        self._curve_vel = NormCurve.linear(0.0, 0.5)
        self._curve_acc = NormCurve.linear(0.0, 2.0)
        self._reload_curves(force=True)
        self._last_reload_check = self.get_clock().now()

        self._signals_path = self.get_parameter('osc_signals_yaml').value
        self._signals_reload_period = self.get_parameter('osc_signals_reload_period_sec').value
        self._signals_mtime = None
        # Start with every built-in enabled, so even if _reload_signals below
        # somehow fails before it sets this, the node sends all 17 addresses —
        # the behaviour from before osc_signals.yaml existed.
        self._enabled_addrs = set(BUILTIN_ADDRESSES)
        self._generic = GenericSourceManager(
            self, self.get_parameter('generic_max_signals').value)
        self._reload_signals(force=True)
        self._last_signals_check = self.get_clock().now()

        targets = [(ip, port)]
        if ip_2:
            targets.append((ip_2, port_2))
        self._sender = OscSender(targets, warn=self.get_logger().warn)

        # Derived kinematics, one state per pose stream.  The arithmetic lives
        # in ee_kinematics.py so mcap_osc_player.py produces identical numbers.
        self._ref = EeState(self._alpha)    # commanded, source: pose_topic
        self._act = EeState(self._alpha)    # actual,    source: actual_pose_topic

        self._send_count = 0
        self._log_every  = max(1, int(rate * 2.0))  # log every ~2 s

        self.create_subscription(PoseStamped, pose_topic, self._odom_cb, 10)
        self.create_subscription(PoseStamped, actual_pose_topic, self._actual_odom_cb, 10)
        self.create_timer(1.0 / rate, self._send_osc)

        targets_str = self._sender.describe()
        self.get_logger().info(
            f'ee_osc_bridge started\n'
            f'  cmd pose topic : {pose_topic}\n'
            f'  act pose topic : {actual_pose_topic}\n'
            f'  OSC targets    : {targets_str}\n'
            f'  send rate      : {rate:.0f} Hz\n'
            f'  EMA alpha      : {self._alpha}\n'
            f'  norm curves    : {self._curves_path}\n'
            f'                   (edit live with curve_editor.py — reloaded every '
            f'{self._curves_reload_period:.0f}s)\n'
            f'  signals        : {self._signals_path}\n'
            f'                   (edit live with signal_editor.py — reloaded every '
            f'{self._signals_reload_period:.0f}s)\n'
            f'                   {len(self._enabled_addrs)}/{len(BUILTIN_SIGNALS)} built-in EE, '
            f'{self._generic.status()}'
        )

    def _reload_curves(self, force: bool = False):
        """(Re)load norm curves from self._curves_path if its mtime changed."""
        path = self._curves_path
        if not path or not os.path.exists(path):
            return
        mtime = os.path.getmtime(path)
        if not force and mtime == self._curves_mtime:
            return
        curves = load_curves(path)
        if 'vel_lin_mag' in curves:
            self._curve_vel = curves['vel_lin_mag']
        if 'accel_lin_mag' in curves:
            self._curve_acc = curves['accel_lin_mag']
        self._curves_mtime = mtime

    def _reload_signals(self, force: bool = False):
        """(Re)load osc_signals.yaml if its mtime changed, and reconcile the
        generic subscriptions to match.

        Same mtime-poll pattern as _reload_curves() above, and deliberately
        total: a missing, empty or malformed file resolves to "all 17 built-ins,
        no generic signals".  This runs inside the 50 Hz timer callback, where
        an exception would kill the whole bridge, so it must never raise.
        """
        path = self._signals_path
        if not path or not os.path.exists(path):
            if self._signals_mtime is not None:     # the file was deleted
                self.get_logger().info(
                    'osc_signals.yaml removed — falling back to all built-in EE signals')
                self._apply_signals(SignalConfig.all_on())
                self._signals_mtime = None
            return

        try:
            mtime = os.stat(path).st_mtime_ns
        except OSError:
            return
        if not force and mtime == self._signals_mtime:
            return
        # Record the mtime before parsing, so a file that fails to load isn't
        # re-parsed (and re-warned about) on every single poll.
        self._signals_mtime = mtime

        try:
            cfg = load_signal_config(path)
        except Exception as exc:
            self.get_logger().warn(
                f'osc_signals.yaml unreadable ({exc}) — keeping the previous config')
            return
        self._apply_signals(cfg)

    def _apply_signals(self, cfg: SignalConfig):
        self._enabled_addrs = cfg.enabled_addresses()
        self._generic.reconcile(cfg.generic)
        for problem in cfg.problems:
            self.get_logger().warn(f'osc_signals.yaml: {problem}')
        self.get_logger().info(
            f'osc_signals reloaded: {len(self._enabled_addrs)}/{len(BUILTIN_SIGNALS)} '
            f'built-in EE, {self._generic.status()}'
        )

    def _odom_cb(self, msg: PoseStamped):
        self._ref.update(*pose_from_msg(msg))

    def _actual_odom_cb(self, msg: PoseStamped):
        self._act.update(*pose_from_msg(msg))

    def _send_osc(self):
        now = self.get_clock().now()
        if (now - self._last_reload_check).nanoseconds * 1e-9 >= self._curves_reload_period:
            self._reload_curves()
            self._last_reload_check = now
        if (now - self._last_signals_check).nanoseconds * 1e-9 >= self._signals_reload_period:
            self._reload_signals()
            self._last_signals_check = now

        send = self._sender.send

        _vel_mag, _accel_mag, act_vel_mag, act_accel_mag = emit_builtin(
            self._ref, self._act, self._enabled_addrs,
            self._curve_vel, self._curve_acc, send,
        )

        # Generic user-selected signals, from the latest-value cache -- same
        # flat (non-bundled) sending, same UDP clients, same tick.  Sent after
        # the built-ins so the EE block's on-the-wire ordering is untouched.
        self._generic.emit(lambda addr, vals: send(addr, [float(v) for v in vals]))

        self._send_count += 1
        if self._send_count % self._log_every == 0:
            pos = self._ref.pos
            act_pos = self._act.pos
            self.get_logger().info(
                f'[#{self._send_count:6d}]  '
                f'cmd_msgs={self._ref.count}  act_msgs={self._act.count}  '
                f'cmd_pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})  '
                f'act_pos=({act_pos[0]:+.3f},{act_pos[1]:+.3f},{act_pos[2]:+.3f})  '
                f'|act_vel|={act_vel_mag:.3f} m/s  '
                f'|act_acc|={act_accel_mag:.3f} m/s²'
            )

    def shutdown_generic(self):
        """Tear down the generic subscriptions before destroy_node().

        destroy_node() would do it anyway; doing it explicitly keeps the
        teardown symmetric with GenericSourceManager's own lifecycle and makes
        the unsubscribe show up in the log.
        """
        self._generic.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = EeOscBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_generic()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
