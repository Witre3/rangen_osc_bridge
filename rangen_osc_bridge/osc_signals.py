#!/usr/bin/env python3
"""
osc_signals — which signals ee_osc_bridge emits, and how to pull an arbitrary
ROS2 message field out of a message as a fixed-length list of floats.

Shared by the ee_osc_bridge ROS2 node and the standalone signal_editor.py GUI,
exactly the way norm_curve.py is shared with curve_editor.py.

Two kinds of signal:

  built-in   The 17 end-effector addresses the bridge has always sent.  Their
             addresses, argument order and argument counts are a FIXED contract
             with the composer's Max patch — osc_signals.yaml only decides
             whether each one is sent at all, never what it looks like.

  generic    Any (topic, field) the user picked in signal_editor.py, forwarded
             as /rangen/<topic>/<field> with a fixed argument count.

No numpy and no rclpy at module level, so `python3 scripts/signal_editor.py`
still opens in a shell with no ROS2 sourced.  rosidl_runtime_py is imported
defensively: without it, field *enumeration* is unavailable but everything
else — including loading, editing and saving the YAML — still works.
"""

import os
import pathlib
import re

import yaml

try:
    from rosidl_runtime_py.utilities import get_message
    ROS_INTROSPECTION_AVAILABLE = True
    ROS_INTROSPECTION_ERROR = ''
except ImportError as exc:          # ROS2 not sourced — the GUI still runs, it
    ROS_INTROSPECTION_AVAILABLE = False   # just can't enumerate message fields.
    ROS_INTROSPECTION_ERROR = str(exc)
    get_message = None


# ── the frozen built-in signal set ───────────────────────────────────────────
#
# (key, OSC address, n_args, human label).  Order is the order ee_osc_bridge
# sends them in, which is also the order signal_editor.py lists them in.  The
# key is the address minus the '/rangen/' prefix, so osc_signals.yaml reads as
# a table of the addresses themselves — a typo'd key can't quietly mute a real
# channel, it just gets logged as unknown.
BUILTIN_SIGNALS = [
    ('ee/pos',                    '/rangen/ee/pos',                    3, 'commanded position  x y z (m)'),
    ('ee/quat',                   '/rangen/ee/quat',                   4, 'commanded orientation  x y z w'),
    ('ee/vel_lin',                '/rangen/ee/vel_lin',                3, 'commanded linear velocity  x y z (m/s)'),
    ('ee/vel_lin/mag',            '/rangen/ee/vel_lin/mag',            1, 'commanded |velocity| (m/s)'),
    ('ee/vel_lin/mag/norm',       '/rangen/ee/vel_lin/mag/norm',       1, "commanded |velocity| 0-1, 'vel_lin_mag' curve"),
    ('ee/vel_ang',                '/rangen/ee/vel_ang',                3, 'commanded angular velocity  x y z (rad/s)'),
    ('ee/accel_lin',              '/rangen/ee/accel_lin',              3, 'commanded linear acceleration  x y z (m/s²)'),
    ('ee/accel_lin/mag',          '/rangen/ee/accel_lin/mag',          1, 'commanded |acceleration| (m/s²)'),
    ('ee/accel_lin/mag/norm',     '/rangen/ee/accel_lin/mag/norm',     1, "commanded |acceleration| 0-1, 'accel_lin_mag' curve"),
    ('ee/act/pos',                '/rangen/ee/act/pos',                3, 'actual position  x y z (m)'),
    ('ee/act/vel_lin',            '/rangen/ee/act/vel_lin',            3, 'actual linear velocity  x y z (m/s)'),
    ('ee/act/vel_lin/mag',        '/rangen/ee/act/vel_lin/mag',        1, 'actual |velocity| (m/s)'),
    ('ee/act/vel_lin/mag/norm',   '/rangen/ee/act/vel_lin/mag/norm',   1, "actual |velocity| 0-1, 'vel_lin_mag' curve"),
    ('ee/act/vel_ang',            '/rangen/ee/act/vel_ang',            3, 'actual angular velocity  x y z (rad/s)'),
    ('ee/act/accel_lin',          '/rangen/ee/act/accel_lin',          3, 'actual linear acceleration  x y z (m/s²)'),
    ('ee/act/accel_lin/mag',      '/rangen/ee/act/accel_lin/mag',      1, 'actual |acceleration| (m/s²)'),
    ('ee/act/accel_lin/mag/norm', '/rangen/ee/act/accel_lin/mag/norm', 1, "actual |acceleration| 0-1, 'accel_lin_mag' curve"),
]

BUILTIN_KEYS = [k for k, _a, _n, _l in BUILTIN_SIGNALS]
BUILTIN_ADDRESSES = frozenset(a for _k, a, _n, _l in BUILTIN_SIGNALS)

# Generic signals may not shadow the EE tree — that whole subtree is the
# composer's contract, and a generic signal landing on one of those addresses
# would silently interleave two different data sources on the same Max inlet.
RESERVED_PREFIX = '/rangen/ee'

# Ceiling on how many floats one generic OSC message may carry.  Not a protocol
# limit — a guard against someone selecting a 640×480 image payload and pushing
# it at 50 Hz.
MAX_ARITY = 16

SCHEMA_VERSION = 1


# ── message type strings ─────────────────────────────────────────────────────

_NUMERIC_PRIMITIVES = {
    'bool', 'byte', 'char',
    'float', 'double', 'float32', 'float64',
    'int8', 'uint8', 'int16', 'uint16',
    'int32', 'uint32', 'int64', 'uint64',
}
_SEQ_RE = re.compile(r'^sequence<\s*([^,>]+?)\s*(?:,\s*(\d+)\s*)?>$')
_ARR_RE = re.compile(r'^(.+?)\[(\d*)\]$')


def split_type(type_str: str):
    """Split an IDL type string into (kind, element_type, fixed_length).

      'sequence<double>'    -> ('array',  'double',              None)
      'double[36]'          -> ('array',  'double',              36)
      'geometry_msgs/Point' -> ('scalar', 'geometry_msgs/Point', None)

    kind is 'array' for anything repeated, 'scalar' otherwise; fixed_length is
    None when the length is only known at runtime (unbounded sequence).
    """
    m = _SEQ_RE.match(type_str)
    if m:
        return 'array', m.group(1), int(m.group(2)) if m.group(2) else None
    m = _ARR_RE.match(type_str)
    if m:
        return 'array', m.group(1), int(m.group(2)) if m.group(2) else None
    return 'scalar', type_str, None


def _try_get_message(type_name: str):
    """get_message() that returns None instead of raising for an unknown type."""
    if not ROS_INTROSPECTION_AVAILABLE:
        return None
    try:
        return get_message(type_name)
    except Exception:
        # Message package not sourced in this shell, or not a message type at
        # all.  Either way it isn't selectable; the caller just skips it.
        return None


# ── field enumeration (for the GUI's field dropdown) ─────────────────────────

class FieldEntry:
    """One selectable field of a message type.

    arity is how many floats it yields, or None when only the runtime message
    knows (an unbounded sequence).
    """

    __slots__ = ('path', 'type_str', 'kind', 'arity')

    def __init__(self, path: str, type_str: str, kind: str, arity):
        self.path = path
        self.type_str = type_str
        self.kind = kind          # 'scalar' | 'vector' | 'array'
        self.arity = arity

    def label(self) -> str:
        """What the GUI combobox shows."""
        n = '?' if self.arity is None else self.arity
        return f'{self.path}   [{n}]   {self.type_str}'

    @staticmethod
    def path_from_label(label: str) -> str:
        """Inverse of label() — the combobox stores labels, the YAML stores paths."""
        return label.split('   ', 1)[0].strip()


def _numeric_leaf_count(cls, depth: int = 0, max_depth: int = 6):
    """How many floats this message type flattens to, or None if it can't.

    None means "don't offer this as a single vector": some leaf is a string, or
    a repeated field whose flattening would be neither fixed-length nor musically
    meaningful (a 36-element covariance matrix on one Max inlet, say).  Callers
    fall back to offering the sub-fields individually.
    """
    if depth > max_depth:
        return None
    total = 0
    for _fname, ftype in cls.get_fields_and_field_types().items():
        kind, elem, _n = split_type(ftype)
        if kind == 'array':
            return None
        if elem in _NUMERIC_PRIMITIVES:
            total += 1
            continue
        if elem in ('string', 'wstring'):
            return None
        sub = _try_get_message(elem)
        if sub is None:
            return None
        n = _numeric_leaf_count(sub, depth + 1, max_depth)
        if n is None:
            return None
        total += n
    return total or None


def enumerate_numeric_fields(type_name: str, max_depth: int = 6):
    """Every numerically-extractable field of a message type, in IDL order.

    Works off the generated Python class alone (get_fields_and_field_types), so
    it needs neither a publisher on the topic nor a single received message —
    which is what lets the GUI offer fields for a topic that isn't up yet.

    Composites whose leaves are all numeric are offered whole as one vector
    (geometry_msgs/Pose -> 7 floats) *and* recursed into, so you can pick either
    'pose' or 'pose.position'.  Arrays of sub-messages are not offered as a unit
    — index into one first, e.g. 'poses[0].pose.position'.
    """
    cls = _try_get_message(type_name)
    if cls is None:
        return []
    out = []

    def walk(c, prefix, depth):
        for fname, ftype in c.get_fields_and_field_types().items():
            path = f'{prefix}.{fname}' if prefix else fname
            kind, elem, n = split_type(ftype)

            if kind == 'array':
                if elem in _NUMERIC_PRIMITIVES:
                    out.append(FieldEntry(path, ftype, 'array', n))
                continue

            if elem in _NUMERIC_PRIMITIVES:
                out.append(FieldEntry(path, ftype, 'scalar', 1))
                continue
            if elem in ('string', 'wstring') or depth >= max_depth:
                continue

            sub = _try_get_message(elem)
            if sub is None:
                continue
            leaves = _numeric_leaf_count(sub)
            if leaves:
                out.append(FieldEntry(path, ftype, 'vector', leaves))
            walk(sub, path, depth + 1)

    walk(cls, '', 0)
    return out


# ── field extraction (runtime, on the node's subscription callback) ──────────

_SEG_RE = re.compile(r'^([A-Za-z_]\w*)(?:\[(\d+)\])?$')


def parse_field_path(path: str):
    """'pose.position' -> [('pose', None), ('position', None)]
       'position[3]'   -> [('position', 3)]

    Raises ValueError on anything that isn't a dotted chain of identifiers with
    optional non-negative integer indices.
    """
    if not path or not path.strip():
        raise ValueError('empty field path')
    segs = []
    for raw in path.split('.'):
        m = _SEG_RE.match(raw.strip())
        if not m:
            raise ValueError(f'bad field-path segment {raw!r} in {path!r}')
        segs.append((m.group(1), int(m.group(2)) if m.group(2) else None))
    return segs


def _flatten(val, out=None):
    """Depth-first flatten of a message / sequence / scalar into floats."""
    out = [] if out is None else out
    if isinstance(val, (bool, int, float)):     # bool is an int subclass — fine
        out.append(float(val))
    elif isinstance(val, (str, bytes)):
        pass                                     # not numeric, contributes nothing
    elif hasattr(val, 'get_fields_and_field_types'):
        # Nested ROS message.  get_fields_and_field_types() returns fields in
        # IDL declaration order, so geometry_msgs/Pose always flattens as
        # (px, py, pz, qx, qy, qz, qw) — the order the Max patch will assume.
        for fname in val.get_fields_and_field_types():
            _flatten(getattr(val, fname), out)
    else:
        try:
            for item in val:                     # list / array.array / ndarray
                _flatten(item, out)
        except TypeError:
            pass
    return out


def extract_floats(msg, segs, arity):
    """Walk `segs` into `msg` and flatten to exactly `arity` floats.

    Short values are zero-padded and long ones truncated, so a signal's OSC
    message shape never changes even if the underlying array does — Max's
    [unpack] downstream depends on a fixed argument count.

    Raises AttributeError / IndexError / TypeError on a mismatch; the caller
    warns once and drops that one signal rather than letting the exception
    reach the node's timer callback.
    """
    val = msg
    for name, idx in segs:
        val = getattr(val, name)
        if idx is not None:
            val = val[idx]

    # Slice before flattening: an unsliced uint8[] image payload would
    # otherwise be walked element by element, 50 times a second.
    if (arity is not None and not isinstance(val, (str, bytes))
            and hasattr(val, '__len__')
            and not hasattr(val, 'get_fields_and_field_types')):
        val = val[:arity]

    vals = _flatten(val)
    if arity is None:
        return vals
    if len(vals) < arity:
        vals.extend([0.0] * (arity - len(vals)))
    return vals[:arity]


def field_length(msg, segs):
    """Runtime length of the sequence at `segs`, or None if it isn't a sequence.

    Used by signal_editor.py to prefill the arity spinbox for an unbounded
    sequence (sensor_msgs/JointState.position is 7 on the Gen3, but nothing in
    the message *type* says so).  Deliberately just a getattr walk and a len():
    no flattening, so it stays cheap even on an image payload.
    """
    val = msg
    for name, idx in segs:
        val = getattr(val, name)
        if idx is not None:
            val = val[idx]
    if isinstance(val, (str, bytes)) or hasattr(val, 'get_fields_and_field_types'):
        return None
    try:
        return len(val)
    except TypeError:
        return None


# ── OSC address derivation ───────────────────────────────────────────────────

# OSC reserves ? * [ ] { } , # and space as pattern-matching characters in
# addresses.  ROS topic names can't contain most of them, but a hand-edited
# address can, and Max would then match it against the wrong route.
_UNSAFE_RE = re.compile(r'[^A-Za-z0-9_.=+~^-]')


def derive_address(topic: str, field_path: str) -> str:
    """/joint_states + position      -> /rangen/joint_states/position
       /gen3/ee_pose  + pose.position -> /rangen/gen3/ee_pose/pose/position
       /joint_states  + position[3]   -> /rangen/joint_states/position/3

    Each dotted component and each index becomes its own address segment, so a
    Max patch can nest [route /rangen/joint_states] -> [route position] -> [route 3].
    """
    parts = [p for p in (topic or '').split('/') if p]   # also eats '//' and trailing '/'
    for seg in (field_path or '').split('.'):
        name, _, idx = seg.partition('[')
        if name:
            parts.append(name)
        if idx:
            parts.append(idx.rstrip(']'))
    parts = [_UNSAFE_RE.sub('_', p) for p in parts if p]
    return '/rangen/' + '/'.join(parts)


def is_reserved_address(address: str) -> bool:
    """True if `address` belongs to the frozen built-in EE tree."""
    return address in BUILTIN_ADDRESSES or address == RESERVED_PREFIX \
        or address.startswith(RESERVED_PREFIX + '/')


# ── config objects ───────────────────────────────────────────────────────────

class GenericSignal:
    """One user-selected (topic, field) -> OSC address forwarding rule."""

    __slots__ = ('topic', 'type', 'field', 'address', 'arity', 'enabled', 'segs')

    def __init__(self, topic, type, field, address='', arity=1, enabled=True):
        self.topic = str(topic or '').strip()
        self.type = str(type or '').strip()
        self.field = str(field or '').strip()
        self.address = str(address or '').strip() or derive_address(self.topic, self.field)
        self.arity = int(arity)
        self.enabled = bool(enabled)
        self.segs = parse_field_path(self.field)     # raises on a malformed path

    def to_dict(self) -> dict:
        return {
            'topic': self.topic, 'type': self.type, 'field': self.field,
            'address': self.address, 'arity': self.arity, 'enabled': self.enabled,
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'GenericSignal':
        return cls(
            topic=d.get('topic', ''), type=d.get('type', ''),
            field=d.get('field', ''), address=d.get('address', ''),
            arity=d.get('arity', 1), enabled=d.get('enabled', True),
        )


class SignalConfig:
    """Parsed osc_signals.yaml: which built-ins are on, plus the generic list.

    `problems` collects human-readable reasons entries were skipped or the file
    was ignored.  The node logs them once per reload rather than raising —
    a broken config must never take the EE stream down with it.
    """

    __slots__ = ('builtin', 'generic', 'problems')

    def __init__(self, builtin: dict, generic: list, problems: list):
        self.builtin = builtin
        self.generic = generic
        self.problems = problems

    @classmethod
    def all_on(cls, problems=None) -> 'SignalConfig':
        """The fallback: all 17 built-ins, no generic signals.

        This is exactly how the bridge behaved before osc_signals.yaml existed,
        and it is what a missing / empty / malformed file resolves to.
        """
        return cls({k: True for k in BUILTIN_KEYS}, [], list(problems or []))

    def enabled_addresses(self) -> set:
        """Built-in addresses to send.  An absent key means enabled, so an old
        or partial file can never silently mute a channel."""
        return {addr for key, addr, _n, _l in BUILTIN_SIGNALS
                if self.builtin.get(key, True)}


def validate_generic(entries, problems):
    """Filter raw generic dicts down to usable GenericSignals.

    Rejects (with a reason appended to `problems`, never an exception): missing
    topic/type/field, an unparseable field path, an out-of-range arity, an
    address in the reserved EE tree, and duplicate addresses — first occurrence
    in file order wins, so hand-editing can't reorder the stream by accident.
    """
    out, seen = [], set()
    for i, raw in enumerate(entries):
        if not isinstance(raw, dict):
            problems.append(f'generic[{i}] is not a mapping — skipped')
            continue
        try:
            sig = GenericSignal.from_dict(raw)
        except (ValueError, TypeError) as exc:
            problems.append(f'generic[{i}] ({raw.get("address", "?")}): {exc} — skipped')
            continue
        if not sig.topic or not sig.type or not sig.field:
            problems.append(f'generic[{i}] ({sig.address}): topic, type and field are all '
                            f'required — skipped')
            continue
        if not sig.address.startswith('/'):
            problems.append(f'generic[{i}]: address {sig.address!r} must start with "/" — skipped')
            continue
        if is_reserved_address(sig.address):
            problems.append(f'address {sig.address} is reserved for the built-in EE '
                            f'signals — entry skipped')
            continue
        if not 1 <= sig.arity <= MAX_ARITY:
            problems.append(f'{sig.address}: arity {sig.arity} outside 1..{MAX_ARITY} — skipped')
            continue
        if sig.address in seen:
            problems.append(f'address {sig.address} used more than once — later entry skipped')
            continue
        seen.add(sig.address)
        out.append(sig)
    return out


def load_signal_config(path) -> SignalConfig:
    """Load osc_signals.yaml.  Never raises.

    A missing, empty, unreadable or malformed file yields SignalConfig.all_on()
    — the 17 built-ins and nothing else, i.e. the bridge's behaviour from before
    this file existed.  Deleting osc_signals.yaml is always safe.
    """
    problems = []
    p = pathlib.Path(path)
    if not p.exists():
        return SignalConfig.all_on()
    try:
        data = yaml.safe_load(p.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        return SignalConfig.all_on([f'unreadable ({exc}) — falling back to all built-ins'])
    if data is None:
        return SignalConfig.all_on(['file is empty — falling back to all built-ins'])
    if not isinstance(data, dict):
        return SignalConfig.all_on(['top level is not a mapping — falling back to all built-ins'])

    version = data.get('version', SCHEMA_VERSION)
    if version != SCHEMA_VERSION:
        problems.append(f'version {version} != {SCHEMA_VERSION} — reading it anyway')

    raw_builtin = data.get('builtin') or {}
    if not isinstance(raw_builtin, dict):
        problems.append("'builtin' is not a mapping — all built-ins left enabled")
        raw_builtin = {}
    builtin = {k: True for k in BUILTIN_KEYS}
    for key, val in raw_builtin.items():
        if key not in builtin:
            problems.append(f'unknown built-in key {key!r} — ignored')
            continue
        builtin[key] = bool(val)

    raw_generic = data.get('generic') or []
    if not isinstance(raw_generic, list):
        problems.append("'generic' is not a list — no generic signals loaded")
        raw_generic = []

    return SignalConfig(builtin, validate_generic(raw_generic, problems), problems)


# ── saving (comment-preserving, atomic) ──────────────────────────────────────

_HEADER = """\
# osc_signals.yaml — which signals ee_osc_bridge emits over OSC.
#
# Edited live with scripts/signal_editor.py.  ee_osc_bridge polls this file's
# mtime and hot-reloads it, so a save applies within ~1 s: no node restart and
# no ROS parameter round-trip.
#
# BACKWARDS COMPATIBILITY: if this file is missing, empty or unparseable, the
# node emits all {n} built-in EE signals and no generic signals — exactly the
# behaviour from before this file existed.  Deleting it is always safe.

version: {v}

# ── Built-in EE signals ──────────────────────────────────────────────────────
# Keys are the OSC address with the '/rangen/' prefix stripped.  The addresses,
# their argument order and count, and the order they are sent in are FIXED —
# the composer's Max patch depends on them.  These switches only decide whether
# each one is sent at all.  A key that is absent here defaults to true.
builtin:
{builtin_lines}
# ── Generic signals ──────────────────────────────────────────────────────────
# Machine-generated by signal_editor.py — this list is rewritten wholesale on
# every save, so comments written inside it will not survive.
#
#   topic    ROS2 topic name (absolute).
#   type     Full message type, 'pkg/msg/Type'.  Stored rather than discovered
#            so the node can subscribe before any publisher exists.
#   field    Dotted path into the message; a segment may carry an index, e.g.
#            'position[3]'.  Must resolve to numbers only.
#   address  OSC address.  Auto-derived as /rangen/<topic>/<field>, freely
#            editable, but may not touch the reserved /rangen/ee/** tree.
#   arity    Exactly this many floats are sent every tick — short values are
#            zero-padded, long ones truncated, so Max always sees a fixed
#            message shape.
#   enabled  false keeps the row here and in the GUI but stops both the OSC
#            output and the ROS subscription.
"""

# Plain YAML scalars that need no quoting: topic names, type names and simple
# field paths.  Anything else (an indexed field like 'position[3]') gets
# single-quoted, since '[' would otherwise open a flow sequence.
_PLAIN_OK = re.compile(r'^[A-Za-z0-9_/.+-]+$')


def _q(s: str) -> str:
    s = '' if s is None else str(s)
    if s and _PLAIN_OK.match(s):
        return s
    return "'" + s.replace("'", "''") + "'"


def _builtin_lines(builtin: dict) -> str:
    width = max(len(k) for k in BUILTIN_KEYS) + 2
    lines = []
    for key, _addr, n, label in BUILTIN_SIGNALS:
        flag = 'true' if builtin.get(key, True) else 'false'
        lines.append(f'  {(key + ":").ljust(width)}{flag.ljust(6)}# {n} arg{"s" if n > 1 else " "}  {label}')
    return '\n'.join(lines) + '\n'


def default_template(builtin=None) -> str:
    """The full default file, generated from BUILTIN_SIGNALS so it can't drift."""
    builtin = builtin or {k: True for k in BUILTIN_KEYS}
    return _HEADER.format(
        n=len(BUILTIN_SIGNALS), v=SCHEMA_VERSION,
        builtin_lines=_builtin_lines(builtin),
    ) + _dump_generic([])


def _dump_generic(generic) -> str:
    """The `generic:` block, including its key line."""
    if not generic:
        return 'generic: []\n'
    out = ['generic:\n']
    for s in generic:
        out.append(f'  - topic:   {_q(s.topic)}\n')
        out.append(f'    type:    {_q(s.type)}\n')
        out.append(f'    field:   {_q(s.field)}\n')
        out.append(f'    address: {_q(s.address)}\n')
        out.append(f'    arity:   {int(s.arity)}\n')
        out.append(f'    enabled: {"true" if s.enabled else "false"}\n')
    return ''.join(out)


_BUILTIN_LINE_RE = re.compile(r'^(  ([\w/]+):\s*)(true|false|True|False)(\s*#.*)?$')
_GENERIC_KEY_RE = re.compile(r'^generic:\s*(\[\s*\])?\s*(#.*)?$')


def save_signal_config(path, builtin: dict, generic) -> pathlib.Path:
    """Rewrite osc_signals.yaml in place, preserving its comments.

    Only the boolean on each known `builtin:` line is rewritten — the same
    line-by-line approach rosbag_inspector.py uses on pkl_field_map.yaml,
    because a yaml.safe_dump round-trip would drop every documentation comment
    in the file.  The `generic:` list is machine-generated, so it is replaced
    wholesale.  A file that doesn't exist (or has lost its `builtin:` block) is
    regenerated from the template.

    Returns the real path written to.
    """
    # Resolve first.  Under `colcon build --symlink-install` the installed
    # config path is a symlink chain back to this source file; writing through
    # it with os.replace() would swap the symlink for a regular file and
    # permanently sever live editing.
    real = pathlib.Path(path).resolve()
    real.parent.mkdir(parents=True, exist_ok=True)

    if not real.exists():
        text = default_template(builtin)
    else:
        text = real.read_text(encoding='utf-8')

    lines = text.splitlines(keepends=True)
    out, section, skipping, seen_keys = [], None, False, set()
    saw_builtin = saw_generic = False

    for line in lines:
        s = line.rstrip('\n')

        if s.rstrip() == 'builtin:':
            section, skipping, saw_builtin = 'builtin', False, True
            out.append(line)
            continue
        if _GENERIC_KEY_RE.match(s):
            # Flush any built-in key the file doesn't have yet (schema grew).
            if section == 'builtin':
                out.extend(_missing_builtin_lines(builtin, seen_keys))
            section, skipping, saw_generic = 'generic', True, True
            out.append(_dump_generic(generic))
            continue

        if s and not s.startswith((' ', '#')):       # dedent to a new top-level key
            if section == 'builtin':
                out.extend(_missing_builtin_lines(builtin, seen_keys))
            section, skipping = None, False

        if skipping:
            continue

        if section == 'builtin':
            m = _BUILTIN_LINE_RE.match(s)
            if m and m.group(2) in builtin:
                seen_keys.add(m.group(2))
                flag = 'true' if builtin[m.group(2)] else 'false'
                # ljust(6) + lstrip'd comment reproduces the template's column
                # layout, so true->false doesn't nudge the comment one char over
                # and show up as noise in `git diff`.
                comment = (m.group(4) or '').lstrip()
                out.append(f'{m.group(1)}{flag.ljust(6) if comment else flag}{comment}\n')
                continue

        out.append(line)

    if section == 'builtin':
        out.extend(_missing_builtin_lines(builtin, seen_keys))

    if not saw_builtin or not saw_generic:
        # The file lost its structure (hand-edited badly, or truncated).  Rather
        # than write something the node would reject, regenerate it in full.
        out = [_HEADER.format(n=len(BUILTIN_SIGNALS), v=SCHEMA_VERSION,
                              builtin_lines=_builtin_lines(builtin)),
               _dump_generic(generic)]

    # Atomic swap, from a temp file in the same directory so it's the same
    # filesystem.  The node stats this file every second; a plain open('w')
    # would give it a window to parse a half-written file.
    tmp = real.with_name(real.name + '.tmp')
    tmp.write_text(''.join(out), encoding='utf-8')
    os.replace(tmp, real)
    return real


def _missing_builtin_lines(builtin: dict, seen: set):
    """Lines for built-in keys the file on disk doesn't carry yet."""
    missing = [k for k in BUILTIN_KEYS if k not in seen]
    if not missing:
        return []
    width = max(len(k) for k in BUILTIN_KEYS) + 2
    out = []
    for key, _addr, n, label in BUILTIN_SIGNALS:
        if key not in missing:
            continue
        flag = 'true' if builtin.get(key, True) else 'false'
        out.append(f'  {(key + ":").ljust(width)}{flag.ljust(6)}# {n} arg{"s" if n > 1 else " "}  {label}\n')
    return out
