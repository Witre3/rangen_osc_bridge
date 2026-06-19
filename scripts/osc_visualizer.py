#!/usr/bin/env python3
"""
Standalone OSC receiver + live data-flow visualizer for ee_osc_bridge.

Does NOT require ROS2 — run it on any machine that can receive UDP from the
robot's network:

  python3 osc_visualizer.py                        # listen on 0.0.0.0:9000
  python3 osc_visualizer.py --port 9001
  python3 osc_visualizer.py --trail 5              # 5-second 3-D trail
  python3 osc_visualizer.py --record take01.jsonl  # record while visualising

The --record flag writes a timestamped JSONL log that osc_to_midi.py can
convert to a MIDI file for Ableton Live.

Dependencies (pip install if missing):
  python-osc   matplotlib   numpy
"""

import argparse
import collections
import importlib
import importlib.util
import json
import pathlib
import sys
import threading
import time

import numpy as np

# ── fix mpl_toolkits namespace conflict between system matplotlib 3.6.x
#    and pip matplotlib 3.11.x.  The system nspkg.pth pre-loads the system
#    path; we prepend the pip path so mplot3d is found there first.
import mpl_toolkits as _mpl_toolkits
_pip_mpl3 = pathlib.Path(
    importlib.util.find_spec('matplotlib').origin
).parent.parent / 'mpl_toolkits'
if _pip_mpl3.exists():
    _p = str(_pip_mpl3)
    if _p in _mpl_toolkits.__path__:
        _mpl_toolkits.__path__.remove(_p)
    _mpl_toolkits.__path__.insert(0, _p)

try:
    import matplotlib
    matplotlib.use('TkAgg')
except Exception:
    pass

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers '3d' projection

try:
    from pythonosc import dispatcher as osc_dispatcher
    from pythonosc.osc_server import ThreadingOSCUDPServer
except ImportError:
    sys.exit("python-osc not found.  Run: pip install python-osc")


# ── config ───────────────────────────────────────────────────────────────────

HISTORY_SECS = 10.0
TRAIL_SECS   = 8.0
N_FADE       = 20
N            = 2000


# ── shared circular buffers ──────────────────────────────────────────────────

_lock  = threading.Lock()

_pos_t = collections.deque(maxlen=N)
_pos_x = collections.deque(maxlen=N)
_pos_y = collections.deque(maxlen=N)
_pos_z = collections.deque(maxlen=N)

_vel_t = collections.deque(maxlen=N)
_vel_x = collections.deque(maxlen=N)
_vel_y = collections.deque(maxlen=N)
_vel_z = collections.deque(maxlen=N)

_acc_t = collections.deque(maxlen=N)
_acc_x = collections.deque(maxlen=N)
_acc_y = collections.deque(maxlen=N)
_acc_z = collections.deque(maxlen=N)

_log   = collections.deque(maxlen=25)
_hz_t  = collections.deque(maxlen=300)

# ── recording ────────────────────────────────────────────────────────────────

_record_fh    = None   # set to open file handle when --record is active
_record_start = None   # wall-clock time of first message written


def _record(now: float, addr: str, *args):
    """Append one JSONL line to the recording file (called from OSC threads)."""
    global _record_start
    fh = _record_fh
    if fh is None:
        return
    if _record_start is None:
        _record_start = now
    fh.write(json.dumps({'t': now, 'addr': addr, 'args': list(args)}) + '\n')


# ── OSC handlers ─────────────────────────────────────────────────────────────

def _handle_pos(addr, x, y, z):
    now = time.time()
    _record(now, addr, float(x), float(y), float(z))
    with _lock:
        _pos_t.append(now); _pos_x.append(float(x))
        _pos_y.append(float(y)); _pos_z.append(float(z))
        _hz_t.append(now)
        _log.append(f'{addr:<33}  {x:+.3f}  {y:+.3f}  {z:+.3f}')


def _handle_vel_lin(addr, x, y, z):
    now = time.time()
    _record(now, addr, float(x), float(y), float(z))
    with _lock:
        _vel_t.append(now); _vel_x.append(float(x))
        _vel_y.append(float(y)); _vel_z.append(float(z))
        _log.append(f'{addr:<33}  {x:+.3f}  {y:+.3f}  {z:+.3f}')


def _handle_accel_lin(addr, x, y, z):
    now = time.time()
    _record(now, addr, float(x), float(y), float(z))
    with _lock:
        _acc_t.append(now); _acc_x.append(float(x))
        _acc_y.append(float(y)); _acc_z.append(float(z))
        _log.append(f'{addr:<33}  {x:+.3f}  {y:+.3f}  {z:+.3f}')


def _handle_generic(addr, *args):
    now = time.time()
    _record(now, addr, *[v for v in args if isinstance(v, (int, float))])
    vals = '  '.join(f'{v:+.4f}' for v in args if isinstance(v, (int, float)))
    with _lock:
        _log.append(f'{addr:<33}  {vals}')


# ── figure layout ─────────────────────────────────────────────────────────────
#
#  ┌──────────────────────┬─────────────────────┐
#  │                      │  vel_lin  x y z      │
#  │  3-D position trail  ├─────────────────────┤
#  │                      │  accel_lin  x y z    │
#  │                      ├─────────────────────┤
#  │                      │  OSC log + rate      │
#  └──────────────────────┴─────────────────────┘

fig = plt.figure(figsize=(16, 8))
fig.suptitle('rangen EE OSC bridge — live monitor', fontsize=12)

gs = gridspec.GridSpec(3, 2, figure=fig,
                       width_ratios=[1.3, 1],
                       height_ratios=[1, 1, 1],
                       hspace=0.50, wspace=0.35)

ax_pos = fig.add_subplot(gs[:, 0], projection='3d')
ax_vel = fig.add_subplot(gs[0, 1])
ax_acc = fig.add_subplot(gs[1, 1])
ax_log = fig.add_subplot(gs[2, 1])

ax_pos.set_title('EE position trail  (m)', pad=8)
ax_pos.set_xlabel('x'); ax_pos.set_ylabel('y'); ax_pos.set_zlabel('z')

ax_vel.set_title('Linear velocity  (m/s)')
ax_vel.set_ylabel('m/s')
ax_vel.set_xlim(-HISTORY_SECS, 0)
ax_vel.grid(True, alpha=0.3)
(ln_vx,) = ax_vel.plot([], [], 'r-', lw=1.3, label='x')
(ln_vy,) = ax_vel.plot([], [], 'g-', lw=1.3, label='y')
(ln_vz,) = ax_vel.plot([], [], 'b-', lw=1.3, label='z')
ax_vel.legend(loc='upper right', fontsize=7)

ax_acc.set_title('Linear acceleration  (m/s²)')
ax_acc.set_ylabel('m/s²')
ax_acc.set_xlim(-HISTORY_SECS, 0)
ax_acc.grid(True, alpha=0.3)
ax_acc.set_xlabel('seconds ago')
(ln_ax,) = ax_acc.plot([], [], 'r-', lw=1.3, label='x')
(ln_ay,) = ax_acc.plot([], [], 'g-', lw=1.3, label='y')
(ln_az,) = ax_acc.plot([], [], 'b-', lw=1.3, label='z')
ax_acc.legend(loc='upper right', fontsize=7)

ax_log.axis('off')
ax_log.set_title('OSC log', loc='left', fontsize=8, pad=2)
txt_log  = ax_log.text(0.0, 1.0, '', transform=ax_log.transAxes,
                       va='top', ha='left', fontsize=7, family='monospace')
txt_rate = fig.text(0.99, 0.005, 'Rate: -- Hz', ha='right', va='bottom', fontsize=9)


# ── helpers ───────────────────────────────────────────────────────────────────

def _window(t_buf, *y_bufs, now, cutoff):
    """Return time-relative and y arrays for the scrolling window."""
    pairs = [(i, t - now) for i, t in enumerate(t_buf) if (t - now) >= cutoff]
    if not pairs:
        return ([], *([[] for _ in y_bufs]))
    idxs, tr = zip(*pairs)
    return (list(tr), *[[b[i] for i in idxs] for b in y_bufs])


def _symmetric_ylim(vals, minimum=0.05):
    lo, hi = min(vals), max(vals)
    span = max(abs(lo), abs(hi), minimum) * 1.25
    return -span, span


# ── animation ─────────────────────────────────────────────────────────────────

def _animate(_frame):
    with _lock:
        pt = list(_pos_t); px = list(_pos_x)
        py = list(_pos_y); pz = list(_pos_z)
        vt = list(_vel_t); vx = list(_vel_x)
        vy = list(_vel_y); vz = list(_vel_z)
        at = list(_acc_t); axl = list(_acc_x)
        ayl = list(_acc_y); azl = list(_acc_z)
        log = list(_log); hz = list(_hz_t)

    now = time.time()
    cut = -HISTORY_SECS

    # ── 3-D fading position trail ─────────────────────────────────────────
    res = _window(pt, px, py, pz, now=now, cutoff=-TRAIL_SECS)
    tr, xr, yr, zr = res[0], res[1], res[2], res[3]

    ax_pos.cla()
    ax_pos.set_title('EE position trail  (m)', pad=8)
    ax_pos.set_xlabel('x'); ax_pos.set_ylabel('y'); ax_pos.set_zlabel('z')

    if len(xr) > 1:
        x = np.array(xr); y = np.array(yr); z = np.array(zr)
        n = len(x)
        for i in range(N_FADE):
            si = i * n // N_FADE
            ei = min((i + 1) * n // N_FADE + 1, n)
            alpha = (i + 1) / N_FADE
            ax_pos.plot3D(x[si:ei], y[si:ei], z[si:ei],
                          color='steelblue', alpha=alpha,
                          linewidth=0.6 + alpha * 1.4)
        ax_pos.scatter([x[-1]], [y[-1]], [z[-1]],
                       color='red', s=50, depthshade=False, zorder=5)
        ax_pos.set_xlim(-1.5, 1.5)
        ax_pos.set_ylim(-1.5, 1.5)
        ax_pos.set_zlim(0.0, 1.5)

    # ── velocity x y z ────────────────────────────────────────────────────
    ax_vel.set_xlim(cut, 0)
    if vt:
        res = _window(vt, vx, vy, vz, now=now, cutoff=cut)
        tr_v, vxw, vyw, vzw = res[0], res[1], res[2], res[3]
        if tr_v:
            ln_vx.set_data(tr_v, vxw)
            ln_vy.set_data(tr_v, vyw)
            ln_vz.set_data(tr_v, vzw)
            ax_vel.set_ylim(*_symmetric_ylim(vxw + vyw + vzw))

    # ── acceleration x y z ────────────────────────────────────────────────
    ax_acc.set_xlim(cut, 0)
    if at:
        res = _window(at, axl, ayl, azl, now=now, cutoff=cut)
        tr_a, axw, ayw, azw = res[0], res[1], res[2], res[3]
        if tr_a:
            ln_ax.set_data(tr_a, axw)
            ln_ay.set_data(tr_a, ayw)
            ln_az.set_data(tr_a, azw)
            ax_acc.set_ylim(*_symmetric_ylim(axw + ayw + azw))

    # ── rate + log ────────────────────────────────────────────────────────
    rate_str = f'Rate: {sum(1 for t in hz if now - t < 1.0)} Hz'
    if _record_fh is not None:
        elapsed = now - (_record_start or now)
        m, s = divmod(int(elapsed), 60)
        rate_str += f'   ● REC  {m:02d}:{s:02d}'
    txt_rate.set_text(rate_str)
    txt_log.set_text('\n'.join(log[-18:]))


# ── OSC server ────────────────────────────────────────────────────────────────

def _start_server(ip: str, port: int):
    d = osc_dispatcher.Dispatcher()
    d.map('/rangen/ee/pos',       _handle_pos)
    d.map('/rangen/ee/vel_lin',   _handle_vel_lin)
    d.map('/rangen/ee/accel_lin', _handle_accel_lin)
    d.set_default_handler(_handle_generic)

    server = ThreadingOSCUDPServer((ip, port), d)
    server.allow_reuse_address = True
    print(f'OSC visualizer listening on {ip}:{port}')
    server.serve_forever()


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    global TRAIL_SECS, _record_fh  # updated from CLI args
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port',   type=int,   default=9000,
                        help='UDP port (default: 9000)')
    parser.add_argument('--ip',                 default='0.0.0.0',
                        help='Bind address (default: 0.0.0.0)')
    parser.add_argument('--trail',  type=float, default=TRAIL_SECS,
                        help=f'3-D trail length in seconds (default: {TRAIL_SECS})')
    parser.add_argument('--record', metavar='FILE', default=None,
                        help='Record all incoming OSC to a JSONL file '
                             '(convert later with osc_to_midi.py)')
    args = parser.parse_args()
    TRAIL_SECS = args.trail

    if args.record:
        _record_fh = open(args.record, 'w')
        print(f'● Recording OSC to {args.record}  (close the window to stop)')

    threading.Thread(target=_start_server, args=(args.ip, args.port), daemon=True).start()

    ani = animation.FuncAnimation(  # noqa: F841
        fig, _animate, interval=80, blit=False, cache_frame_data=False,
    )
    try:
        plt.show()
    finally:
        if _record_fh is not None:
            _record_fh.flush()
            _record_fh.close()
            n_lines = pathlib.Path(args.record).stat().st_size
            print(f'● Recording saved → {args.record}  ({n_lines} bytes)')


if __name__ == '__main__':
    main()
