#!/usr/bin/env python3
"""
mcap_sources — the MCAP half of ee_osc_bridge's generic signal layer.

Same job as generic_sources.py, same reconcile / cache / emit shape and the same
warn-once failure map, but the messages come out of a recorded .mcap file
instead of a live DDS subscription.  A row in osc_signals.yaml therefore
produces the same OSC address carrying the same numbers whether it is played
from the robot or from a bag on a Mac — which is the whole point of splitting
the field extraction into osc_signals.py in the first place.  `extract_floats`
and `field_length` are imported from there unchanged.

Two things the MCAP side gets for free that the rclpy side has to work for:

  * No rosidl.  generic_sources._start() has to resolve the message class with
    get_message(type_name), which fails for any custom message whose package
    isn't sourced.  Here the channel already names its type and carries its
    schema inline, so mcap_ros2 builds a decoder from the file itself — a
    ranger_msgs field works on a machine that has never heard of ranger_msgs.
  * No QoS.  There is no publisher to match; the bag either has the topic or
    it doesn't.

Nothing here imports rclpy, and nothing imports numpy.
"""

import time

try:                                    # package layout (repo / colcon install)
    from .osc_signals import extract_floats, field_length
except ImportError:                     # flat layout (macOS bundle app/ dir)
    from osc_signals import extract_floats, field_length


class LatestValue:
    """Last extracted floats for one signal, and when they arrived."""

    __slots__ = ('vals', 'stamp')

    def __init__(self, vals, stamp):
        self.vals = vals
        self.stamp = stamp


class McapSourceManager:
    """Field extraction and latest-value cache for the user-selected signals.

    Built once per bag, since a file's channel list can't change underneath us.
    `bind()` takes the signals from osc_signals.yaml and the bag's channels and
    works out which signals this particular bag can actually serve.
    """

    def __init__(self, warn=None, max_signals: int = 32):
        self._warn = warn or (lambda msg: None)
        self._max = max(0, int(max_signals))
        self._signals = []      # usable signals, file order
        self._by_topic = {}     # topic -> [GenericSignal]
        self._cache = {}        # address -> LatestValue
        self._failed = {}       # topic or address -> reason (warn-once)

    def bind(self, signals, channels_by_topic) -> None:
        """Match `signals` against the bag's `{topic: (channel, schema)}`.

        A signal naming a topic the bag doesn't contain, or naming a different
        message type than the bag recorded, is reported once and skipped — the
        rest keep working.  Same tolerance as the node: a bad config row must
        never take the EE stream down with it.
        """
        wanted = [s for s in signals if s.enabled]
        if len(wanted) > self._max:
            dropped = len(wanted) - self._max
            self._warn(f'{dropped} generic signal(s) over the limit of {self._max} — '
                       f'ignoring the extras')
            wanted = wanted[:self._max]

        usable = []
        for s in wanted:
            entry = channels_by_topic.get(s.topic)
            if entry is None:
                self._fail(s.topic, f'not recorded in this bag — signal {s.address} skipped')
                continue
            _channel, schema = entry
            if s.type and schema.name != s.type:
                self._fail(s.address,
                           f'type mismatch: config says {s.type}, bag has {schema.name}')
                continue
            usable.append(s)

        self._signals = usable
        self._by_topic = {}
        for s in usable:
            self._by_topic.setdefault(s.topic, []).append(s)

    def topics(self):
        """Topics that must be CDR-decoded during playback.

        The player decodes only these (plus the two pose topics) and forwards
        everything else to Foxglove as raw bytes — the bags are several GB of
        mostly camera frames, and decoding them all would not keep up.
        """
        return set(self._by_topic)

    # ── data path ────────────────────────────────────────────────────────────

    def on_message(self, topic: str, msg) -> None:
        """Feed one decoded message; extract every field configured for it."""
        now = time.monotonic()
        for s in self._by_topic.get(topic, ()):
            try:
                vals = extract_floats(msg, s.segs, s.arity)
            except Exception as exc:
                # Wrong field for this message type, or an index past the end of
                # a short array.  Mute this one signal and warn once.
                self._fail(s.address, f'field {s.field!r}: {exc.__class__.__name__}: {exc}')
                continue
            self._cache[s.address] = LatestValue(vals, now)

    def emit(self, send) -> None:
        """Send every cached signal, once.

        A signal that has never received a message emits nothing at all: zero
        would tell Max that a dead sensor reads exactly 0.0, which is worse than
        silence.  Once a message has arrived the last value is held, matching
        the built-in EE addresses.
        """
        for s in self._signals:
            lv = self._cache.get(s.address)
            if lv is not None:
                send(s.address, lv.vals)

    def reset(self) -> None:
        """Drop cached values, for --loop restarting the bag."""
        self._cache = {}

    # ── reporting ────────────────────────────────────────────────────────────

    def _fail(self, key: str, reason: str) -> None:
        if self._failed.get(key) == reason:
            return
        self._failed[key] = reason
        self._warn(f'generic OSC {key}: {reason}')

    def failures(self) -> dict:
        return dict(self._failed)

    def status(self) -> str:
        live = sum(1 for s in self._signals if s.address in self._cache)
        return (f'{len(self._signals)} generic signal(s) on {len(self._by_topic)} topic(s), '
                f'{live} receiving, {len(self._failed)} unavailable')
