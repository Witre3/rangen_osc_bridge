#!/usr/bin/env python3
"""
generic_sources — the rclpy half of ee_osc_bridge's generic signal layer.

Owns one subscription per (topic, message type) the user selected in
signal_editor.py, extracts the configured field out of every message, and
caches the result so the node's 50 Hz timer can emit it alongside the built-in
EE addresses.

Why cache instead of sending on message arrival:

  * The composer's Max patch sees one uniform stream.  Every built-in address
    ticks at exactly 50 Hz; a generic signal firing at its topic's native rate
    would put 1 Hz and 1 kHz addresses in the same patch, and the [line]/smoothing
    objects downstream are tuned for 50 Hz.
  * Bounded network cost.  A 1 kHz IMU forwarded verbatim would flood the WiFi
    link to the composer's laptop -- the same link ee_osc_bridge already counts
    dropped packets on.  Cached, each signal costs exactly 50 pkt/s per target.
  * All OSC sending stays on the timer thread, so no lock is needed around the
    node's UDP clients.

Structure follows rangen_graph's ProbeManager (rangen_graph/probe.py) -- same
reconcile / _start / _stop / shutdown shape and the same warn-once failure map.
It is copied rather than imported because rangen_osc_bridge is a separate git
repo and must not gain a build dependency on rangen_graph.
"""

import time

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from .osc_signals import extract_floats, field_length


def subscription_qos(publisher_qos) -> QoSProfile:
    """Build a subscription QoS guaranteed to match `publisher_qos`.

    Copied from rangen_graph/probe.py:probe_qos().  DDS matches on
    "requested <= offered", so mirroring the publisher's reliability and
    durability is compatible by construction.  This matters: rclpy's default
    subscription is RELIABLE, and a RELIABLE subscription receives *nothing*
    from a BEST_EFFORT publisher -- a silent failure that looks exactly like a
    dead topic.  Deadline, lifespan and liveliness stay at their permissive
    defaults; requesting less than the publisher offers is always fine.
    """
    profile = QoSProfile(depth=10)
    profile.history = HistoryPolicy.KEEP_LAST
    profile.reliability = ReliabilityPolicy.BEST_EFFORT
    profile.durability = DurabilityPolicy.VOLATILE

    if publisher_qos is not None:
        try:
            profile.reliability = publisher_qos.reliability
            profile.durability = publisher_qos.durability
            if publisher_qos.history == HistoryPolicy.KEEP_LAST:
                profile.depth = max(1, min(int(publisher_qos.depth), 10))
        except Exception:
            pass
    return profile


class LatestValue:
    """Last extracted floats for one signal, and when they arrived.

    `natural` is the field's runtime sequence length (None if it isn't a
    sequence) — signal_editor.py uses it to prefill the arity spinbox for an
    unbounded sequence, whose length the message *type* doesn't reveal.
    """

    __slots__ = ('vals', 'stamp', 'natural')

    def __init__(self, vals, stamp, natural=None):
        self.vals = vals
        self.stamp = stamp
        self.natural = natural


class GenericSourceManager:
    """Subscriptions and latest-value cache for the user-selected signals.

    One subscription per (topic, type), shared by every signal reading a field
    of that topic -- subscribing three times to /joint_states to read three
    fields would triple the network cost for nothing.
    """

    def __init__(self, node, max_signals: int = 32):
        self._node = node
        self._max = max(0, int(max_signals))
        self._subs = {}         # (topic, type) -> Subscription
        self._signals = []      # enabled signals, file order
        self._by_topic = {}     # (topic, type) -> [GenericSignal]
        self._cache = {}        # address -> LatestValue
        self._failed = {}       # topic or address -> reason (warn-once)

    # ── lifecycle ────────────────────────────────────────────────────────────

    def reconcile(self, signals) -> None:
        """Make the live subscriptions match `signals` exactly.

        Called only from the node's 50 Hz timer callback, i.e. the executor
        thread that owns the node -- the same contract ProbeManager.sync()
        relies on.  create_subscription() and destroy_subscription() both wake
        the executor, and Subscription.destroy() goes through
        destroy_when_not_in_use(), so creating and destroying subscriptions
        mid-callback is safe and needs no deferred-destroy queue.
        """
        wanted = [s for s in signals if s.enabled]
        if len(wanted) > self._max:
            dropped = len(wanted) - self._max
            wanted = wanted[:self._max]
            self._node.get_logger().warn(
                f'generic OSC: {dropped} signal(s) over the generic_max_signals '
                f'limit of {self._max} — ignoring the extras')

        desired = {(s.topic, s.type) for s in wanted}
        for key in desired - set(self._subs):
            self._start(*key)
        for key in set(self._subs) - desired:
            self._stop(key)

        # Rebuild the dispatch tables.  Cache entries for addresses that are no
        # longer configured are dropped, so a removed signal can't keep emitting
        # a stale value.
        self._signals = wanted
        self._by_topic = {}
        for s in wanted:
            self._by_topic.setdefault((s.topic, s.type), []).append(s)

        live = {s.address for s in wanted}
        for addr in set(self._cache) - live:
            del self._cache[addr]
        # Clear per-signal failures too, so fixing a bad field path in the GUI
        # gives that signal a fresh chance instead of staying muted until restart.
        for addr in set(self._failed) - live - {t for t, _ty in desired}:
            del self._failed[addr]

    def _start(self, topic: str, type_name: str) -> None:
        try:
            msg_cls = get_message(type_name)
        except Exception as exc:
            # Custom message whose package isn't sourced in this shell -- the
            # same failure mode probe.py handles, and just as non-fatal.
            self._fail(topic, f'type unavailable: {type_name} ({exc.__class__.__name__})')
            return

        qos = subscription_qos(self._publisher_qos(topic))
        try:
            sub = self._node.create_subscription(
                msg_cls, topic,
                lambda msg, k=(topic, type_name): self._on_msg(k, msg),
                qos)
        except Exception as exc:
            self._fail(topic, f'subscribe failed: {exc}')
            return

        self._subs[(topic, type_name)] = sub
        self._failed.pop(topic, None)
        self._node.get_logger().info(f'generic OSC: subscribed {topic} [{type_name}]')

    def _stop(self, key) -> None:
        sub = self._subs.pop(key, None)
        if sub is not None:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
            self._node.get_logger().info(f'generic OSC: unsubscribed {key[0]}')

    def shutdown(self) -> None:
        for key in list(self._subs):
            self._stop(key)
        self._signals = []
        self._by_topic = {}
        self._cache = {}

    def _publisher_qos(self, topic: str):
        """The first publisher's QoS profile for `topic`, or None if nobody
        is publishing yet (in which case the permissive default is used and
        DDS will match whatever shows up later)."""
        try:
            infos = self._node.get_publishers_info_by_topic(topic)
        except Exception:
            return None
        return infos[0].qos_profile if infos else None

    # ── data path ────────────────────────────────────────────────────────────

    def _on_msg(self, key, msg) -> None:
        now = time.monotonic()
        for s in self._by_topic.get(key, ()):
            try:
                vals = extract_floats(msg, s.segs, s.arity)
            except Exception as exc:
                # Wrong field for this message type, or an index past the end of
                # a short array.  Mute this one signal and warn once -- never let
                # it propagate, since this callback runs on the node's executor.
                self._fail(s.address, f'field {s.field!r}: {exc.__class__.__name__}: {exc}')
                continue
            try:
                natural = field_length(msg, s.segs)
            except Exception:
                natural = None
            self._cache[s.address] = LatestValue(vals, now, natural)

    def emit(self, send) -> None:
        """Send every cached signal, once, via the node's raw OSC sender.

        A signal that has never received a message emits nothing at all: zero
        would tell Max that a dead sensor reads exactly 0.0, which is worse than
        silence.  Once a message has arrived the last value is held indefinitely,
        matching the built-in EE addresses (whose position also holds after the
        publisher stops).
        """
        for s in self._signals:
            lv = self._cache.get(s.address)
            if lv is not None:
                send(s.address, lv.vals)

    # ── reporting ────────────────────────────────────────────────────────────

    def _fail(self, key: str, reason: str) -> None:
        if self._failed.get(key) == reason:
            return                      # already warned; don't spam once per tick
        self._failed[key] = reason
        self._node.get_logger().warn(f'generic OSC {key}: {reason}')

    def snapshot(self) -> dict:
        """{address: (vals, age_sec, natural_len)} — what signal_editor.py shows
        in its live-value column, and the failure reason for signals that have
        one (as (None, None, None) plus a lookup in failures())."""
        now = time.monotonic()
        return {addr: (lv.vals, now - lv.stamp, lv.natural)
                for addr, lv in self._cache.items()}

    def failures(self) -> dict:
        """{topic or address: reason} for everything currently unavailable."""
        return dict(self._failed)

    def status(self) -> str:
        live = sum(1 for s in self._signals if s.address in self._cache)
        return (f'{len(self._signals)} generic signal(s), {len(self._subs)} subscription(s), '
                f'{live} receiving, {len(self._failed)} unavailable')
