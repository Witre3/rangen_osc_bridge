#!/usr/bin/env python3
"""
norm_curve — shared breakpoint-curve normalization used by both the
ee_osc_bridge ROS2 node and the standalone curve_editor.py GUI.

A NormCurve maps a raw scalar (e.g. |vel|, |accel| in SI units) to 0-1
through a piecewise curve defined by control points at (x, y) in the unit
square, x=0..1 spanning [input_min, input_max].  Each point also carries a
`bend` in [-1, 1] that warps the segment running from that point to the
next one, like a DAW clip-fade handle:

  bend == 0   linear
  bend  > 0   ease-in  (stays low, then rises sharply near the next point)
  bend  < 0   ease-out (rises sharply, then flattens toward the next point)

The last point's bend is unused (no segment follows it).

No numpy/ROS dependency — importable standalone by a plain `python3` script.
"""

import pathlib

import yaml

_BEND_GAIN = 4.0  # bend=+-1 -> exponent 16 or 1/16; feels like a full DAW fade


class NormPoint:
    __slots__ = ('x', 'y', 'bend')

    def __init__(self, x: float, y: float, bend: float = 0.0):
        self.x = max(0.0, min(1.0, float(x)))
        self.y = max(0.0, min(1.0, float(y)))
        self.bend = max(-1.0, min(1.0, float(bend)))

    def to_dict(self) -> dict:
        return {'x': self.x, 'y': self.y, 'bend': self.bend}

    @classmethod
    def from_dict(cls, d: dict) -> 'NormPoint':
        return cls(d['x'], d['y'], d.get('bend', 0.0))


class NormCurve:
    """Breakpoint curve mapping [input_min, input_max] -> [0, 1]."""

    def __init__(self, points=None, input_min: float = 0.0, input_max: float = 1.0):
        self.input_min = float(input_min)
        self.input_max = float(input_max)
        self.points = list(points) if points else [
            NormPoint(0.0, 0.0), NormPoint(1.0, 1.0),
        ]
        self._sort()

    def _sort(self):
        self.points.sort(key=lambda p: p.x)

    # ── evaluation ───────────────────────────────────────────────────────

    def evaluate(self, raw_value: float) -> float:
        span = self.input_max - self.input_min
        if span <= 1e-12:
            x = 0.0
        else:
            x = (raw_value - self.input_min) / span
            x = max(0.0, min(1.0, x))

        pts = self.points
        if len(pts) < 2:
            return pts[0].y if pts else 0.0

        for i in range(len(pts) - 1):
            p0, p1 = pts[i], pts[i + 1]
            if x <= p1.x or i == len(pts) - 2:
                seg = p1.x - p0.x
                if seg < 1e-9:
                    return p1.y
                t = (x - p0.x) / seg
                t = max(0.0, min(1.0, t))
                k = 2.0 ** (p0.bend * _BEND_GAIN)
                t_warped = t ** k
                return p0.y + (p1.y - p0.y) * t_warped
        return pts[-1].y

    # ── point editing (used by the GUI) ─────────────────────────────────

    def add_point(self, x: float, y: float, bend: float = 0.0) -> NormPoint:
        p = NormPoint(x, y, bend)
        self.points.append(p)
        self._sort()
        return p

    def remove_point(self, p: NormPoint):
        # Endpoints (x=0 and x=1) anchor the domain and cannot be removed.
        if p in self.points and 0.0 < p.x < 1.0:
            self.points.remove(p)

    # ── (de)serialization ────────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            'input_min': self.input_min,
            'input_max': self.input_max,
            'points': [p.to_dict() for p in self.points],
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'NormCurve':
        points = [NormPoint.from_dict(pd) for pd in d.get('points', [])]
        return cls(
            points=points or None,
            input_min=d.get('input_min', 0.0),
            input_max=d.get('input_max', 1.0),
        )

    @classmethod
    def linear(cls, input_min: float, input_max: float) -> 'NormCurve':
        return cls([NormPoint(0.0, 0.0), NormPoint(1.0, 1.0)], input_min, input_max)


def load_curves(path) -> dict:
    """Load {channel_name: NormCurve} from a YAML file. Missing file -> {}."""
    path = pathlib.Path(path)
    if not path.exists():
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    curves = data.get('curves', {})
    return {name: NormCurve.from_dict(cd) for name, cd in curves.items()}


def save_curves(path, curves: dict):
    """Write {channel_name: NormCurve} to a YAML file."""
    path = pathlib.Path(path)
    data = {'curves': {name: c.to_dict() for name, c in curves.items()}}
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
