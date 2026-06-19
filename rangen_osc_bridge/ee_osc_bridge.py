#!/usr/bin/env python3
"""
ee_osc_bridge — ROS2 node that forwards end-effector kinematics as OSC.

Subscribes to the IGPS odometry topic (nav_msgs/Odometry).  Velocity and
acceleration are NOT published by the upstream node, so they are derived
here via finite differences and smoothed with an EMA filter.

OSC address schema (all per-timer-tick bundle):
  /rangen/ee/pos            x y z          (m, in map frame)
  /rangen/ee/quat           x y z w
  /rangen/ee/vel_lin        x y z          (m/s, EMA-filtered)
  /rangen/ee/vel_lin/mag    scalar
  /rangen/ee/vel_lin/mag/norm  0–1 per config range
  /rangen/ee/vel_ang        x y z          (rad/s, world frame)
  /rangen/ee/accel_lin      x y z          (m/s², EMA-filtered)
  /rangen/ee/accel_lin/mag  scalar
  /rangen/ee/accel_lin/mag/norm  0–1 per config range
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from pythonosc import udp_client
from pythonosc.osc_bundle_builder import OscBundleBuilder, IMMEDIATELY
from pythonosc.osc_message_builder import OscMessageBuilder


def _quat_mult(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Quaternion multiplication [x, y, z, w] × [x, y, z, w]."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _clip01(v: float) -> float:
    return max(0.0, min(1.0, v))


class EeOscBridge(Node):

    def __init__(self):
        super().__init__('ee_osc_bridge')

        self.declare_parameter('osc_target_ip', '127.0.0.1')
        self.declare_parameter('osc_target_port', 9000)
        self.declare_parameter('send_rate_hz', 50.0)
        # EMA alpha: 1.0 = raw, 0.0 = frozen.  ~0.1–0.3 gives musical smoothing.
        self.declare_parameter('smoothing_alpha', 0.15)
        self.declare_parameter('pose_topic',
                               '/ground_truth/odometry_igps/gen3_robotiq_85_tool_link')
        self.declare_parameter('norm_vel_lin_mag_min', 0.0)
        self.declare_parameter('norm_vel_lin_mag_max', 0.5)
        self.declare_parameter('norm_accel_lin_mag_min', 0.0)
        self.declare_parameter('norm_accel_lin_mag_max', 2.0)

        ip   = self.get_parameter('osc_target_ip').value
        port = self.get_parameter('osc_target_port').value
        rate = self.get_parameter('send_rate_hz').value
        self._alpha = self.get_parameter('smoothing_alpha').value
        pose_topic  = self.get_parameter('pose_topic').value

        self._vel_norm_min   = self.get_parameter('norm_vel_lin_mag_min').value
        self._vel_norm_max   = self.get_parameter('norm_vel_lin_mag_max').value
        self._acc_norm_min   = self.get_parameter('norm_accel_lin_mag_min').value
        self._acc_norm_max   = self.get_parameter('norm_accel_lin_mag_max').value

        self._client = udp_client.SimpleUDPClient(ip, port)

        # Derived-kinematics state
        self._pos  = np.zeros(3)
        self._quat = np.array([0.0, 0.0, 0.0, 1.0])
        self._vel_lin  = np.zeros(3)
        self._vel_ang  = np.zeros(3)
        self._accel_lin = np.zeros(3)

        self._prev_pos  = None
        self._prev_quat = None
        self._prev_t    = None

        self.create_subscription(Odometry, pose_topic, self._odom_cb, 10)
        self.create_timer(1.0 / rate, self._send_osc)

        self.get_logger().info(
            f'ee_osc_bridge: {pose_topic} → OSC {ip}:{port} @ {rate:.0f} Hz  '
            f'alpha={self._alpha}'
        )

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        pos  = np.array([p.x, p.y, p.z])
        quat = np.array([o.x, o.y, o.z, o.w])

        if self._prev_t is not None:
            dt = t - self._prev_t
            if dt > 1e-6:
                self._update_kinematics(pos, quat, dt)

        self._pos  = pos
        self._quat = quat
        self._prev_pos  = pos
        self._prev_quat = quat
        self._prev_t    = t

    def _update_kinematics(self, pos: np.ndarray, quat: np.ndarray, dt: float):
        alpha = self._alpha

        # Linear velocity: finite difference + EMA
        raw_vl = (pos - self._prev_pos) / dt
        new_vl = alpha * raw_vl + (1.0 - alpha) * self._vel_lin

        # Linear acceleration: derivative of smoothed velocity + EMA
        raw_al = (new_vl - self._vel_lin) / dt
        self._accel_lin = alpha * raw_al + (1.0 - alpha) * self._accel_lin

        self._vel_lin = new_vl

        # Angular velocity from quaternion finite difference.
        # q_rel = q_curr ⊗ q_prev_conj  ≈  [ω·dt/2, w≈1] for small rotations.
        q_prev_conj = np.array([
            -self._prev_quat[0], -self._prev_quat[1],
            -self._prev_quat[2],  self._prev_quat[3],
        ])
        q_rel = _quat_mult(quat, q_prev_conj)
        if q_rel[3] < 0:
            q_rel = -q_rel  # shortest-path convention
        raw_va = 2.0 * q_rel[:3] / dt
        self._vel_ang = alpha * raw_va + (1.0 - alpha) * self._vel_ang

    def _send_osc(self):
        pos = self._pos
        quat = self._quat
        vl = self._vel_lin
        va = self._vel_ang
        al = self._accel_lin

        vel_mag   = float(np.linalg.norm(vl))
        accel_mag = float(np.linalg.norm(al))

        vel_norm_range   = self._vel_norm_max - self._vel_norm_min
        accel_norm_range = self._acc_norm_max - self._acc_norm_min
        vel_norm   = _clip01((vel_mag   - self._vel_norm_min)   / max(vel_norm_range, 1e-9))
        accel_norm = _clip01((accel_mag - self._acc_norm_min) / max(accel_norm_range, 1e-9))

        builder = OscBundleBuilder(IMMEDIATELY)

        def _add(addr: str, *vals):
            mb = OscMessageBuilder(address=addr)
            for v in vals:
                mb.add_arg(float(v))
            builder.add_content(mb.build())

        _add('/rangen/ee/pos',             pos[0],  pos[1],  pos[2])
        _add('/rangen/ee/quat',            quat[0], quat[1], quat[2], quat[3])
        _add('/rangen/ee/vel_lin',         vl[0],   vl[1],   vl[2])
        _add('/rangen/ee/vel_lin/mag',     vel_mag)
        _add('/rangen/ee/vel_lin/mag/norm', vel_norm)
        _add('/rangen/ee/vel_ang',         va[0],   va[1],   va[2])
        _add('/rangen/ee/accel_lin',       al[0],   al[1],   al[2])
        _add('/rangen/ee/accel_lin/mag',   accel_mag)
        _add('/rangen/ee/accel_lin/mag/norm', accel_norm)

        self._client.send(builder.build())


def main(args=None):
    rclpy.init(args=args)
    node = EeOscBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
