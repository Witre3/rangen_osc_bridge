#!/usr/bin/env python3
"""
Standalone OSC receiver + live data-flow visualizer for ee_osc_bridge.

Does NOT require ROS2 — run it on any machine that can receive UDP from the
robot's network:

  python3 osc_visualizer.py              # listen on 0.0.0.0:9000
  python3 osc_visualizer.py --port 9001
  python3 osc_visualizer.py --port 9000 --ip 0.0.0.0

Dependencies (pip install if missing):
  python-osc   matplotlib   numpy
"""

import argparse
import collections
import sys
import threading
import time

import numpy as np

try:
    import matplotlib
    matplotlib.use('TkAgg')
except Exception:
    pass  # fall back to whatever backend is available

import matplotlib.pyplot as plt
import matplotlib.animation as animation

try:
    from pythonosc import dispatcher as osc_dispatcher
    from pythonosc.osc_server import ThreadingOSCUDPServer
except ImportError:
    sys.exit("python-osc not found.  Run: pip install python-osc")


# ── shared circular buffers ──────────────────────────────────────────────────

HISTORY_SECS = 10.0
N = 2000

_lock = threading.Lock()

_pos_t  = collections.deque(maxlen=N)
_pos_x  = collections.deque(maxlen=N)
_pos_y  = collections.deque(maxlen=N)
_pos_z  = collections.deque(maxlen=N)

_vel_t  = collections.deque(maxlen=N)
_vel_m  = collections.deque(maxlen=N)

_acc_t  = collections.deque(maxlen=N)
_acc_m  = collections.deque(maxlen=N)

_log    = collections.deque(maxlen=25)
_hz_t   = collections.deque(maxlen=300)   # wall-clock times of /pos arrivals


# ── OSC handlers ─────────────────────────────────────────────────────────────

def _handle_pos(addr, x, y, z):
    now = time.time()
    with _lock:
        _pos_t.append(now)
        _pos_x.append(float(x))
        _pos_y.append(float(y))
        _pos_z.append(float(z))
        _hz_t.append(now)
        _log.append(f'{addr:<33}  {x:+.3f}  {y:+.3f}  {z:+.3f}')


def _handle_vel_mag(addr, mag):
    now = time.time()
    with _lock:
        _vel_t.append(now)
        _vel_m.append(float(mag))
        _log.append(f'{addr:<33}  {mag:.4f}')


def _handle_acc_mag(addr, mag):
    now = time.time()
    with _lock:
        _acc_t.append(now)
        _acc_m.append(float(mag))
        _log.append(f'{addr:<33}  {mag:.4f}')


def _handle_generic(addr, *args):
    vals = '  '.join(
        f'{v:+.4f}' for v in args if isinstance(v, (int, float))
    )
    with _lock:
        _log.append(f'{addr:<33}  {vals}')


# ── matplotlib figure ────────────────────────────────────────────────────────

fig, axes = plt.subplots(4, 1, figsize=(13, 9), gridspec_kw={'height_ratios': [2, 1.5, 1.5, 2]})
fig.suptitle('rangen EE OSC bridge — live monitor', fontsize=12)

ax_pos, ax_vel, ax_acc, ax_log = axes

ax_pos.set_title('Position  (m, map frame)')
ax_pos.set_ylabel('m')
ax_pos.set_xlim(-HISTORY_SECS, 0)
ax_pos.grid(True, alpha=0.3)

ax_vel.set_title('Linear velocity magnitude  (m/s)')
ax_vel.set_ylabel('m/s')
ax_vel.set_xlim(-HISTORY_SECS, 0)
ax_vel.set_ylim(0, 0.5)
ax_vel.grid(True, alpha=0.3)

ax_acc.set_title('Linear acceleration magnitude  (m/s²)')
ax_acc.set_ylabel('m/s²')
ax_acc.set_xlim(-HISTORY_SECS, 0)
ax_acc.set_ylim(0, 2.0)
ax_acc.grid(True, alpha=0.3)
ax_acc.set_xlabel('seconds ago')

ax_log.axis('off')
ax_log.set_title('OSC message log', loc='left', fontsize=9)

(ln_px,) = ax_pos.plot([], [], 'r-',  lw=1.2, label='x')
(ln_py,) = ax_pos.plot([], [], 'g-',  lw=1.2, label='y')
(ln_pz,) = ax_pos.plot([], [], 'b-',  lw=1.2, label='z')
ax_pos.legend(loc='upper right', fontsize=8)

(ln_vel,) = ax_vel.plot([], [], color='#cc66ff', lw=1.4)
(ln_acc,) = ax_acc.plot([], [], color='#ff8800', lw=1.4)

txt_log  = ax_log.text(0.0, 1.0, '', transform=ax_log.transAxes,
                       va='top', ha='left', fontsize=7.5, family='monospace',
                       wrap=True)
txt_rate = fig.text(0.99, 0.01, 'Rate: -- Hz', ha='right', va='bottom', fontsize=9)

plt.tight_layout(rect=[0, 0.0, 1, 0.97])


def _scrolling(t_buf, y_buf, now, cutoff):
    """Return (t_rel, y) arrays clipped to the scrolling window."""
    pairs = [(t - now, y) for t, y in zip(t_buf, y_buf) if (t - now) >= cutoff]
    if not pairs:
        return [], []
    tr, yr = zip(*pairs)
    return list(tr), list(yr)


def _animate(_frame):
    with _lock:
        pt  = list(_pos_t);  px  = list(_pos_x)
        py  = list(_pos_y);  pz  = list(_pos_z)
        vt  = list(_vel_t);  vm  = list(_vel_m)
        at  = list(_acc_t);  am  = list(_acc_m)
        log = list(_log)
        hz  = list(_hz_t)

    now    = time.time()
    cutoff = -HISTORY_SECS

    # ── position ──────────────────────────────────────────────────────────
    if pt:
        tr, xr = _scrolling(pt, px, now, cutoff)
        _,  yr = _scrolling(pt, py, now, cutoff)
        _,  zr = _scrolling(pt, pz, now, cutoff)
        if tr:
            ln_px.set_data(tr, xr)
            ln_py.set_data(tr, yr)
            ln_pz.set_data(tr, zr)
            ax_pos.set_xlim(cutoff, 0)
            yall = xr + yr + zr
            pad  = max((max(yall) - min(yall)) * 0.15, 0.02)
            ax_pos.set_ylim(min(yall) - pad, max(yall) + pad)

    # ── velocity ──────────────────────────────────────────────────────────
    if vt:
        tr, yr = _scrolling(vt, vm, now, cutoff)
        if tr:
            ln_vel.set_data(tr, yr)
            ax_vel.set_xlim(cutoff, 0)
            ax_vel.set_ylim(0, max(max(yr) * 1.15, 0.05))

    # ── acceleration ─────────────────────────────────────────────────────
    if at:
        tr, yr = _scrolling(at, am, now, cutoff)
        if tr:
            ln_acc.set_data(tr, yr)
            ax_acc.set_xlim(cutoff, 0)
            ax_acc.set_ylim(0, max(max(yr) * 1.15, 0.1))

    # ── rate ──────────────────────────────────────────────────────────────
    recent = [t for t in hz if now - t < 1.0]
    txt_rate.set_text(f'Rate: {len(recent)} Hz')

    # ── log ───────────────────────────────────────────────────────────────
    txt_log.set_text('\n'.join(log[-20:]))

    return ln_px, ln_py, ln_pz, ln_vel, ln_acc, txt_rate, txt_log


# ── OSC server (background thread) ──────────────────────────────────────────

def _start_server(ip: str, port: int):
    d = osc_dispatcher.Dispatcher()
    d.map('/rangen/ee/pos',           _handle_pos)
    d.map('/rangen/ee/vel_lin/mag',   _handle_vel_mag)
    d.map('/rangen/ee/accel_lin/mag', _handle_acc_mag)
    d.set_default_handler(_handle_generic)

    server = ThreadingOSCUDPServer((ip, port), d)
    server.allow_reuse_address = True
    print(f'OSC visualizer listening on {ip}:{port}')
    server.serve_forever()


# ── entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=9000, help='UDP port to listen on (default: 9000)')
    parser.add_argument('--ip',   default='0.0.0.0',     help='IP to bind on (default: 0.0.0.0)')
    args = parser.parse_args()

    t = threading.Thread(target=_start_server, args=(args.ip, args.port), daemon=True)
    t.start()

    ani = animation.FuncAnimation(   # noqa: F841  (kept alive by reference)
        fig, _animate, interval=80, blit=False, cache_frame_data=False,
    )
    plt.show()


if __name__ == '__main__':
    main()
