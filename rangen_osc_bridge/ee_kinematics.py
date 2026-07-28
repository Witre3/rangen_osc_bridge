#!/usr/bin/env python3
"""
ee_kinematics — the built-in EE signal math and OSC sending, with no ROS2.

Split out of ee_osc_bridge.py so the exact same code produces the exact same
17 addresses whether the pose came from a live DDS subscription (the node) or
from a recorded MCAP file (mcap_osc_player.py, which runs on a Mac with no ROS2
installed at all).  Two implementations of this arithmetic would drift, and the
addresses and their argument order are a fixed contract with the composer's Max
patch — so there is only one.

No rclpy and no numpy at module level, matching osc_signals.py and
norm_curve.py.  The vectors here are 3- and 4-element tuples: at 50 Hz the
pure-Python arithmetic is free, and dropping numpy keeps the macOS bundle to
wheels that are pure Python or already built for both Mac architectures.

The operation order is a deliberate transcription of the numpy version it
replaces, so the float results are bit-identical to what the node emitted
before.
"""

import math


def quat_mult(a, b):
    """Quaternion multiplication [x, y, z, w] × [x, y, z, w]."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def norm3(v):
    """Euclidean norm of a 3-vector — numpy.linalg.norm's sqrt(dot(v, v))."""
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


class EeState:
    """Derived kinematics for one end-effector pose stream.

    One instance per stream: the bridge runs two, commanded (/ee_pose_ref) and
    actual (/ee_pose_actual).  Velocity and acceleration are not published by
    any upstream node, so they are finite-differenced here and smoothed with an
    EMA — alpha 1.0 is raw, 0.0 is frozen, ~0.1–0.3 is musically useful.

    Timestamps come from the message header, not from arrival time, so replaying
    a bag yields the same numbers the live run did.
    """

    def __init__(self, alpha: float = 0.15):
        self.alpha = float(alpha)

        self.pos = (0.0, 0.0, 0.0)
        self.quat = (0.0, 0.0, 0.0, 1.0)
        self.vel_lin = (0.0, 0.0, 0.0)
        self.vel_ang = (0.0, 0.0, 0.0)
        self.accel_lin = (0.0, 0.0, 0.0)

        self._prev_pos = None
        self._prev_quat = None
        self._prev_t = None

        self.count = 0

    def update(self, pos, quat, t: float) -> None:
        """Feed one pose sample.  `pos` is (x, y, z), `quat` is (x, y, z, w),
        `t` is the header stamp in seconds."""
        pos = (float(pos[0]), float(pos[1]), float(pos[2]))
        quat = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))

        if self._prev_t is not None:
            dt = t - self._prev_t
            if dt > 1e-6:
                self._integrate(pos, quat, dt)

        self.pos = pos
        self.quat = quat
        self._prev_pos = pos
        self._prev_quat = quat
        self._prev_t = t
        self.count += 1

    def _integrate(self, pos, quat, dt: float) -> None:
        a = self.alpha
        b = 1.0 - a
        pp = self._prev_pos
        vl = self.vel_lin

        # Linear velocity: finite difference + EMA
        new_vl = tuple(
            a * ((pos[i] - pp[i]) / dt) + b * vl[i] for i in range(3)
        )

        # Linear acceleration: derivative of the smoothed velocity + EMA
        self.accel_lin = tuple(
            a * ((new_vl[i] - vl[i]) / dt) + b * self.accel_lin[i] for i in range(3)
        )

        self.vel_lin = new_vl

        # Angular velocity from a quaternion finite difference.
        # q_rel = q_curr ⊗ q_prev_conj ≈ [ω·dt/2, w≈1] for small rotations.
        pq = self._prev_quat
        q_rel = quat_mult(quat, (-pq[0], -pq[1], -pq[2], pq[3]))
        if q_rel[3] < 0:
            q_rel = tuple(-c for c in q_rel)     # shortest-path convention
        self.vel_ang = tuple(
            a * (2.0 * q_rel[i] / dt) + b * self.vel_ang[i] for i in range(3)
        )

    @property
    def vel_lin_mag(self) -> float:
        return norm3(self.vel_lin)

    @property
    def accel_lin_mag(self) -> float:
        return norm3(self.accel_lin)


class OscSender:
    """Flat OSC/UDP sending to one or more targets, with tolerant error handling.

    Flat messages, never an OscBundleBuilder envelope: Max/MSP's plain
    udpreceive→route chain does not parse the '#bundle' framing without an
    OSC-route/o.route object, which was silently discarding every packet on the
    composer's end.

    A failed send drops that one packet instead of propagating.  In the node
    this runs inside the 50 Hz timer callback, where an exception would stop the
    whole bridge — including the other, unaffected target.
    """

    def __init__(self, targets, warn=None):
        from pythonosc import udp_client       # deferred: keeps import cost off

        self.targets = list(targets)
        self._clients = [udp_client.SimpleUDPClient(ip, port) for ip, port in self.targets]
        self._warn = warn or (lambda msg: None)
        self.errors = 0

    def send(self, addr: str, args) -> None:
        for client in self._clients:
            try:
                client.send_message(addr, args)
            except OSError as e:
                # Transient, e.g. a WiFi hiccup causing EWOULDBLOCK on sendto.
                self.errors += 1
                if self.errors % 100 == 1:
                    self._warn(f'OSC send failed, dropping packet '
                               f'(#{self.errors} so far): {e}')

    def describe(self) -> str:
        return ', '.join(f'{ip}:{port}' for ip, port in self.targets)


def emit_builtin(ref: EeState, act: EeState, enabled_addrs, curve_vel, curve_acc, send):
    """Send the 17 built-in EE addresses, once, in their fixed order.

    `enabled_addrs` is the set from osc_signals.yaml; it is the *only* gate on
    this block.  The addresses, their argument order and their on-the-wire
    ordering cannot drift from what the composer's Max patch expects.

    Returns (vel_mag, accel_mag, act_vel_mag, act_accel_mag) for logging.
    """
    def _add(addr, *vals):
        if addr in enabled_addrs:
            send(addr, [float(v) for v in vals])

    vel_mag = ref.vel_lin_mag
    accel_mag = ref.accel_lin_mag
    vel_norm = curve_vel.evaluate(vel_mag)
    accel_norm = curve_acc.evaluate(accel_mag)

    p, q, vl, va, al = ref.pos, ref.quat, ref.vel_lin, ref.vel_ang, ref.accel_lin

    # Commanded EE
    _add('/rangen/ee/pos',                p[0],  p[1],  p[2])
    _add('/rangen/ee/quat',               q[0],  q[1],  q[2], q[3])
    _add('/rangen/ee/vel_lin',            vl[0], vl[1], vl[2])
    _add('/rangen/ee/vel_lin/mag',        vel_mag)
    _add('/rangen/ee/vel_lin/mag/norm',   vel_norm)
    _add('/rangen/ee/vel_ang',            va[0], va[1], va[2])
    _add('/rangen/ee/accel_lin',          al[0], al[1], al[2])
    _add('/rangen/ee/accel_lin/mag',      accel_mag)
    _add('/rangen/ee/accel_lin/mag/norm', accel_norm)

    act_vel_mag = act.vel_lin_mag
    act_accel_mag = act.accel_lin_mag
    act_vel_norm = curve_vel.evaluate(act_vel_mag)
    act_accel_norm = curve_acc.evaluate(act_accel_mag)

    ap, avl, ava, aal = act.pos, act.vel_lin, act.vel_ang, act.accel_lin

    # Actual (measured) EE
    _add('/rangen/ee/act/pos',                ap[0],  ap[1],  ap[2])
    _add('/rangen/ee/act/vel_lin',            avl[0], avl[1], avl[2])
    _add('/rangen/ee/act/vel_lin/mag',        act_vel_mag)
    _add('/rangen/ee/act/vel_lin/mag/norm',   act_vel_norm)
    _add('/rangen/ee/act/vel_ang',            ava[0], ava[1], ava[2])
    _add('/rangen/ee/act/accel_lin',          aal[0], aal[1], aal[2])
    _add('/rangen/ee/act/accel_lin/mag',      act_accel_mag)
    _add('/rangen/ee/act/accel_lin/mag/norm', act_accel_norm)

    return vel_mag, accel_mag, act_vel_mag, act_accel_mag


def pose_from_msg(msg):
    """(pos, quat, t) out of a geometry_msgs/PoseStamped.

    Works on a real ROS2 message and on an mcap_ros2-decoded one alike — both
    expose the same attribute path, which is what makes the MCAP player able to
    reuse this whole module.
    """
    p = msg.pose.position
    o = msg.pose.orientation
    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w), t
