#!/usr/bin/env python3
"""
curve_editor — interactive GUI for tuning the 0-1 normalization curves that
ee_osc_bridge applies to /rangen/ee/vel_lin/mag and /rangen/ee/accel_lin/mag.

Does NOT require ROS2 — run it on any machine that can receive the raw
(unnormalized) magnitude OSC messages the bridge sends:

  python3 curve_editor.py                         # listen on 0.0.0.0:9002
  python3 curve_editor.py --port 9002
  python3 curve_editor.py -c my_norm_curves.yaml

While tuning, point the bridge's OSC target at this tool's port (see
README.md), so the live playhead on each curve shows you where real motion
actually falls. ee_osc_bridge hot-reloads the curve file, so saved changes
take effect immediately without restarting the node.

Mouse / keyboard:
  drag a point            move it (endpoints are x-locked to the domain edges)
  drag a segment's curve   bend that segment (like an audio clip fade handle)
  double-click empty space add a new breakpoint
  right-click a point      delete it (endpoints can't be deleted)
  's'                      save both curves to the config file
  'r'                      reset both curves to linear

Dependencies (pip install if missing):
  python-osc   matplotlib   pyyaml
"""

import argparse
import math
import pathlib
import sys
import threading
import time

# Make `rangen_osc_bridge.norm_curve` importable whether or not the
# workspace has been colcon-sourced (this script has no ROS2 dependency).
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from rangen_osc_bridge.norm_curve import NormCurve, load_curves, save_curves  # noqa: E402

try:
    import matplotlib
    matplotlib.use('TkAgg')
except Exception:
    pass

import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.animation as animation  # noqa: E402

try:
    from pythonosc import dispatcher as osc_dispatcher
    from pythonosc.osc_server import ThreadingOSCUDPServer
except ImportError:
    sys.exit("python-osc not found.  Run: pip install python-osc")


_DEFAULT_CONFIG = pathlib.Path(__file__).parent.parent / 'config' / 'norm_curves.yaml'

_HIT_PX = 10           # pixel radius for grabbing a point or bend handle
_SAVE_FLASH_SEC = 1.5
_STALE_SEC = 2.0        # no OSC for this long -> playhead greys out

_CHANNELS = [
    ('vel_lin_mag',   '/rangen/ee/vel_lin/mag',   '|vel|  (m/s)',  0.0, 0.5),
    ('accel_lin_mag', '/rangen/ee/accel_lin/mag', '|accel|  (m/s²)', 0.0, 2.0),
]


class CurvePanel:
    """One draggable breakpoint-curve editor living in a single Axes."""

    def __init__(self, ax, name: str, label: str, curve: NormCurve):
        self.ax = ax
        self.name = name
        self.curve = curve
        self.drag = None  # ('point', idx) | ('bend', seg_idx) | None

        ax.set_title(f'{name}   ({label})', fontsize=10)
        ax.set_xlim(curve.input_min, curve.input_max)
        ax.set_ylim(-0.05, 1.05)
        ax.set_xlabel(label, fontsize=8)
        ax.set_ylabel('normalized output', fontsize=8)
        ax.grid(True, alpha=0.3)

        (self.curve_line,) = ax.plot([], [], '-', color='steelblue', lw=2, zorder=3)
        self.point_scatter = ax.scatter([], [], s=60, color='steelblue',
                                         edgecolor='white', zorder=5, picker=False)
        self.bend_scatter = ax.scatter([], [], s=28, color='darkorange',
                                        edgecolor='white', zorder=4, alpha=0.85)
        self.playhead_v = ax.axvline(curve.input_min, color='crimson', lw=1,
                                      alpha=0.0, zorder=2)
        (self.playhead_dot,) = ax.plot([], [], 'o', color='crimson', ms=8,
                                        alpha=0.0, zorder=6)
        self.hud = ax.text(0.02, 0.96, '', transform=ax.transAxes, fontsize=8,
                            va='top', ha='left', family='monospace')

    # ── geometry helpers ─────────────────────────────────────────────────

    def _raw_x(self, i: int) -> float:
        p = self.curve.points[i]
        return self.curve.input_min + p.x * (self.curve.input_max - self.curve.input_min)

    def _seg_mid_raw(self, seg_idx: int):
        p0 = self.curve.points[seg_idx]
        p1 = self.curve.points[seg_idx + 1]
        x_mid_frac = (p0.x + p1.x) / 2.0
        raw_x = self.curve.input_min + x_mid_frac * (self.curve.input_max - self.curve.input_min)
        y_mid = self.curve.evaluate(raw_x)
        return raw_x, y_mid

    def _px_dist(self, data_xy, event):
        x0, y0 = self.ax.transData.transform(data_xy)
        return math.hypot(event.x - x0, event.y - y0)

    def hit_test(self, event):
        if event.inaxes is not self.ax:
            return None
        for i, p in enumerate(self.curve.points):
            if self._px_dist((self._raw_x(i), p.y), event) <= _HIT_PX:
                return ('point', i)
        for seg_idx in range(len(self.curve.points) - 1):
            if self._px_dist(self._seg_mid_raw(seg_idx), event) <= _HIT_PX:
                return ('bend', seg_idx)
        return None

    # ── editing ──────────────────────────────────────────────────────────

    def move_point(self, idx: int, raw_x: float, y: float):
        pts = self.curve.points
        span = self.curve.input_max - self.curve.input_min
        x = 0.0 if span <= 0 else (raw_x - self.curve.input_min) / span
        x = max(0.0, min(1.0, x))
        eps = 1e-3
        if idx == 0:
            x = 0.0
        elif idx == len(pts) - 1:
            x = 1.0
        else:
            x = max(pts[idx - 1].x + eps, min(pts[idx + 1].x - eps, x))
        pts[idx].x = x
        pts[idx].y = max(0.0, min(1.0, y))

    def bend_segment(self, seg_idx: int, drag_y: float):
        p0 = self.curve.points[seg_idx]
        p1 = self.curve.points[seg_idx + 1]
        drag_y = max(0.001, min(0.999, drag_y))
        span_y = p1.y - p0.y
        if abs(span_y) < 1e-6:
            return
        ratio = (drag_y - p0.y) / span_y
        ratio = max(0.02, min(0.98, ratio))
        k = math.log(ratio) / math.log(0.5)
        k = max(2.0 ** -4, min(2.0 ** 4, k))
        bend = math.log2(k) / 4.0
        p0.bend = max(-1.0, min(1.0, bend))

    def add_point(self, raw_x: float, y: float):
        span = self.curve.input_max - self.curve.input_min
        x = 0.0 if span <= 0 else (raw_x - self.curve.input_min) / span
        self.curve.add_point(max(0.02, min(0.98, x)), max(0.0, min(1.0, y)))

    def delete_point(self, idx: int):
        self.curve.remove_point(self.curve.points[idx])

    def reset_linear(self):
        self.curve.points = [self.curve.points[0].__class__(0.0, 0.0),
                              self.curve.points[0].__class__(1.0, 1.0)]

    # ── drawing ──────────────────────────────────────────────────────────

    def redraw(self, raw_value, last_update_ts):
        curve = self.curve
        n = 200
        span = curve.input_max - curve.input_min
        xs = [curve.input_min + span * i / (n - 1) for i in range(n)]
        ys = [curve.evaluate(x) for x in xs]
        self.curve_line.set_data(xs, ys)

        pxs = [self._raw_x(i) for i in range(len(curve.points))]
        pys = [p.y for p in curve.points]
        self.point_scatter.set_offsets(list(zip(pxs, pys)))

        bxs, bys = [], []
        for seg_idx in range(len(curve.points) - 1):
            bx, by = self._seg_mid_raw(seg_idx)
            bxs.append(bx); bys.append(by)
        self.bend_scatter.set_offsets(list(zip(bxs, bys)) if bxs else [])

        if raw_value is not None and (time.time() - last_update_ts) < _STALE_SEC:
            out = curve.evaluate(raw_value)
            self.playhead_v.set_xdata([raw_value, raw_value])
            self.playhead_v.set_alpha(0.6)
            self.playhead_dot.set_data([raw_value], [out])
            self.playhead_dot.set_alpha(0.9)
            self.hud.set_text(f'raw={raw_value:6.3f}\nnorm={out:5.3f}')
        else:
            self.playhead_v.set_alpha(0.0)
            self.playhead_dot.set_alpha(0.0)
            self.hud.set_text('no OSC data')


# ── shared live-value state (updated by the OSC server thread) ─────────────

_lock = threading.Lock()
_raw_values = {name: None for name, *_ in _CHANNELS}
_last_update = {name: 0.0 for name, *_ in _CHANNELS}

_save_flash_until = 0.0


def _make_handler(name):
    def _handler(addr, value):
        with _lock:
            _raw_values[name] = float(value)
            _last_update[name] = time.time()
    return _handler


def _start_osc_server(ip: str, port: int):
    d = osc_dispatcher.Dispatcher()
    for name, addr, *_ in _CHANNELS:
        d.map(addr, _make_handler(name))
    server = ThreadingOSCUDPServer((ip, port), d)
    server.allow_reuse_address = True
    print(f'curve_editor listening on {ip}:{port}  '
          f'(point ee_osc_bridge\'s osc_target_port here while tuning)')
    server.serve_forever()


# ── main ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=9002,
                        help='UDP port to listen for raw magnitude OSC (default: 9002)')
    parser.add_argument('--ip', default='0.0.0.0', help='Bind address (default: 0.0.0.0)')
    parser.add_argument('-c', '--config', type=pathlib.Path, default=_DEFAULT_CONFIG,
                        help=f'Curve YAML to load/save (default: {_DEFAULT_CONFIG})')
    args = parser.parse_args()

    existing = load_curves(args.config) if args.config.exists() else {}

    fig, axes = plt.subplots(1, len(_CHANNELS), figsize=(13, 5.5))
    fig.suptitle('rangen normalization curve editor  —  drag points/curves, '
                  "'s' save, 'r' reset, right-click deletes a point", fontsize=10)

    panels = {}
    for ax, (name, addr, label, dmin, dmax) in zip(axes, _CHANNELS):
        curve = existing.get(name) or NormCurve.linear(dmin, dmax)
        panels[name] = CurvePanel(ax, name, label, curve)

    status_text = fig.text(0.5, 0.965, '', ha='center', fontsize=10, color='seagreen')

    def _panel_for_event(event):
        for p in panels.values():
            if event.inaxes is p.ax:
                return p
        return None

    def on_press(event):
        panel = _panel_for_event(event)
        if panel is None or event.xdata is None:
            return
        if event.button == 3:  # right click -> delete nearest point
            hit = panel.hit_test(event)
            if hit and hit[0] == 'point':
                panel.delete_point(hit[1])
            return
        if event.dblclick:
            hit = panel.hit_test(event)
            if hit is None:
                panel.add_point(event.xdata, event.ydata)
            return
        panel.drag = panel.hit_test(event)

    def on_motion(event):
        panel = _panel_for_event(event)
        if panel is None or panel.drag is None or event.xdata is None:
            return
        kind, idx = panel.drag
        if kind == 'point':
            panel.move_point(idx, event.xdata, event.ydata)
        elif kind == 'bend':
            panel.bend_segment(idx, event.ydata)

    def on_release(event):
        for p in panels.values():
            p.drag = None

    def on_key(event):
        global _save_flash_until
        if event.key == 's':
            save_curves(args.config, {name: p.curve for name, p in panels.items()})
            _save_flash_until = time.time() + _SAVE_FLASH_SEC
            print(f'Saved curves -> {args.config}')
        elif event.key == 'r':
            for p in panels.values():
                p.reset_linear()

    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    fig.canvas.mpl_connect('button_release_event', on_release)
    fig.canvas.mpl_connect('key_press_event', on_key)

    threading.Thread(target=_start_osc_server, args=(args.ip, args.port), daemon=True).start()

    def _animate(_frame):
        with _lock:
            raw = dict(_raw_values)
            upd = dict(_last_update)
        for name, p in panels.items():
            p.redraw(raw.get(name), upd.get(name, 0.0))
        status_text.set_text('SAVED ✓' if time.time() < _save_flash_until else '')

    ani = animation.FuncAnimation(  # noqa: F841
        fig, _animate, interval=40, blit=False, cache_frame_data=False,
    )
    plt.tight_layout(rect=(0, 0, 1, 0.93))
    plt.show()


if __name__ == '__main__':
    main()
