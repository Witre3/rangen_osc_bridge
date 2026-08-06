#!/usr/bin/env python3
"""
Standalone OSC receiver + live data-flow visualizer for ee_osc_bridge.

Does NOT require ROS2 — run it on any machine that can receive UDP from the
robot's network:

  python3 osc_visualizer.py                        # listen on 127.0.0.1:9004
  python3 osc_visualizer.py --port 9001
  python3 osc_visualizer.py --trail 5              # 5-second 3-D trail
  python3 osc_visualizer.py --record take01        # record from launch (stem only)

Panels: 3-D EE trail, linear velocity, linear acceleration, arm state
(/rangen/reach and /rangen/elbow_state from the state_interpreter node, with
the /rangen/reach/discrete level as a colour-coded badge), and the OSC log.

Press the  REC  button in the window to start/stop recording interactively.
Each session writes two files to logs/:
  osc_YYYYMMDD_HHMMSS.csv    -- t,addr,arg0,arg1,arg2  (Max/MSP, Excel, pandas)
  osc_YYYYMMDD_HHMMSS.jsonl  -- timestamped JSON lines (osc_to_midi.py, replay)

Replay to Max/MSP (or any OSC client) with osc_replay.py:
  python3 osc_replay.py logs/osc_20260702_143000.jsonl
Max patch: [udpreceive 9004] -> [oscparse] -> [route /rangen/ee/pos ...]

Dependencies (pip install if missing):
  python-osc   matplotlib   numpy
"""

import argparse
import collections
import importlib
import importlib.util
import json
import pathlib
import signal
import sys
import threading
import time

import numpy as np

# -- fix mpl_toolkits namespace conflict between system matplotlib 3.6.x
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
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 -- registers '3d' projection

try:
    from pythonosc import dispatcher as osc_dispatcher
    from pythonosc.osc_server import ThreadingOSCUDPServer
except ImportError:
    sys.exit("python-osc not found.  Run: pip install python-osc")


# -- config -------------------------------------------------------------------

HISTORY_SECS = 10.0
TRAIL_SECS   = 8.0
N_FADE       = 20
N            = 2000
LOG_DIR      = pathlib.Path(__file__).parent.parent / 'logs'

# Where /rangen/reach/discrete steps, drawn as dashed lines on the state panel.
# Defaults match reach_near_m / reach_far_m in config/state_interpreter.yaml;
# --reach-near / --reach-far override them if that config is retuned.
REACH_NEAR_M = 0.60
REACH_FAR_M  = 0.95

# Colour per discrete reach level: near, mid, far.
LEVEL_COLORS = ('#2e7d32', '#f9a825', '#c62828')


# -- shared circular buffers --------------------------------------------------

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

# state_interpreter's signals.  Separate time buffers per address: they are
# three different OSC messages, and one can be missing (e.g. the node isn't
# running) without stalling the others.
_reach_t = collections.deque(maxlen=N)
_reach_v = collections.deque(maxlen=N)

_elbow_t = collections.deque(maxlen=N)
_elbow_v = collections.deque(maxlen=N)

_level   = None     # latest /rangen/reach/discrete, None until one arrives

_log   = collections.deque(maxlen=25)
_hz_t  = collections.deque(maxlen=300)

# -- recording ----------------------------------------------------------------
# Both files are opened/closed together.  OSC threads read these atomically
# (single name-lookup under the GIL) without acquiring _lock.

_record_fh     = None   # JSONL  -- for osc_to_midi.py and osc_replay.py
_record_csv_fh = None   # CSV    -- for Excel, pandas
_record_txt_fh = None   # space-separated .txt -- for Max text object + zl slice
_record_start  = None   # wall-clock time of first message written
_recording     = False


def _record(now: float, addr: str, *args):
    """Write one event to JSONL, CSV, and Max txt (called from OSC threads)."""
    global _record_start
    fh     = _record_fh       # single atomic read -- safe without extra locking
    csv_fh = _record_csv_fh
    txt_fh = _record_txt_fh
    if fh is None:
        return
    if _record_start is None:
        _record_start = now
    # JSONL
    fh.write(json.dumps({'t': now, 'addr': addr, 'args': list(args)}) + '\n')
    # CSV -- fixed 3-arg columns; pad with empty strings if fewer args
    padded = list(args) + ['', '']
    csv_fh.write(f'{now:.6f},{addr},{padded[0]},{padded[1]},{padded[2]}\n')
    # space-separated -- readable by Max text object; zl slice 1 -> addr, right -> args
    txt_fh.write(f'{now:.6f} {addr} {padded[0]} {padded[1]} {padded[2]}\n')


def _open_recording(stem: str):
    """Open JSONL, CSV, and Max txt under logs/<stem>."""
    global _record_fh, _record_csv_fh, _record_txt_fh, _record_start, _recording
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    jsonl_path = LOG_DIR / f'{stem}.jsonl'
    csv_path   = LOG_DIR / f'{stem}.csv'
    txt_path   = LOG_DIR / f'{stem}.txt'
    _record_fh     = open(jsonl_path, 'w')
    _record_csv_fh = open(csv_path,   'w')
    _record_txt_fh = open(txt_path,   'w')
    _record_csv_fh.write('t,addr,arg0,arg1,arg2\n')
    _record_start  = None
    _recording     = True
    return jsonl_path, csv_path, txt_path


def _close_recording():
    """Flush and close all three recording files."""
    global _record_fh, _record_csv_fh, _record_txt_fh, _recording
    fh     = _record_fh
    csv_fh = _record_csv_fh
    txt_fh = _record_txt_fh
    _record_fh     = None   # stop OSC threads writing before close
    _record_csv_fh = None
    _record_txt_fh = None
    _recording     = False
    for f in (fh, csv_fh, txt_fh):
        if f is not None:
            f.flush()
            f.close()
            p = pathlib.Path(f.name)
            print(f'Saved -> {p}  ({p.stat().st_size} bytes)')


# -- OSC handlers -------------------------------------------------------------

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


def _handle_reach(addr, v):
    now = time.time()
    _record(now, addr, float(v))
    with _lock:
        _reach_t.append(now); _reach_v.append(float(v))
        _log.append(f'{addr:<33}  {v:+.3f}')


def _handle_elbow_state(addr, v):
    now = time.time()
    _record(now, addr, float(v))
    with _lock:
        _elbow_t.append(now); _elbow_v.append(float(v))
        _log.append(f'{addr:<33}  {v:+.3f}')


def _handle_reach_discrete(addr, v):
    global _level
    now = time.time()
    _record(now, addr, float(v))
    with _lock:
        _level = int(v)
        _log.append(f'{addr:<33}  {int(v)}')


def _handle_generic(addr, *args):
    now = time.time()
    numeric = [v for v in args if isinstance(v, (int, float))]
    _record(now, addr, *numeric)
    vals = '  '.join(f'{v:+.4f}' for v in numeric)
    with _lock:
        _log.append(f'{addr:<33}  {vals}')


# -- figure layout ------------------------------------------------------------
#
#  +----------------------+---------------------+
#  |                      |  vel_lin  x y z      |
#  |                      +---------------------+
#  |  3-D position trail  |  accel_lin  x y z    |
#  |                      +---------------------+
#  |                      |  reach / elbow_state |
#  |                      +---------------------+
#  |                      |  OSC log + rate      |
#  +----------------------+---------------------+
#  | [REC]                                       |  <- button strip
#  +----------------------------------------------+

fig = plt.figure(figsize=(16, 8))
fig.subplots_adjust(bottom=0.12)
fig.suptitle('rangen EE OSC bridge -- live monitor', fontsize=12)

gs = gridspec.GridSpec(4, 2, figure=fig,
                       width_ratios=[1.3, 1],
                       height_ratios=[1, 1, 1, 1],
                       hspace=0.75, wspace=0.35)

ax_pos   = fig.add_subplot(gs[:, 0], projection='3d')
ax_vel   = fig.add_subplot(gs[0, 1])
ax_acc   = fig.add_subplot(gs[1, 1])
ax_state = fig.add_subplot(gs[2, 1])
ax_log   = fig.add_subplot(gs[3, 1])

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

ax_acc.set_title('Linear acceleration  (m/s^2)')
ax_acc.set_ylabel('m/s^2')
ax_acc.set_xlim(-HISTORY_SECS, 0)
ax_acc.grid(True, alpha=0.3)
(ln_ax,) = ax_acc.plot([], [], 'r-', lw=1.3, label='x')
(ln_ay,) = ax_acc.plot([], [], 'g-', lw=1.3, label='y')
(ln_az,) = ax_acc.plot([], [], 'b-', lw=1.3, label='z')
ax_acc.legend(loc='upper right', fontsize=7)

# state_interpreter: /rangen/reach, /rangen/elbow_state, /rangen/reach/discrete.
# Both curves are in metres, so they share one axis; the dashed lines are where
# the discrete output steps, which is what makes a take's level changes readable
# at a glance.
ax_state.set_title('Arm state  (m)', loc='left')    # left, so the level badge
ax_state.set_ylabel('m')                            # can share the same strip
ax_state.set_xlim(-HISTORY_SECS, 0)
ax_state.set_xlabel('seconds ago')
ax_state.grid(True, alpha=0.3)
(ln_reach,) = ax_state.plot([], [], color='tab:purple', lw=1.5, label='reach')
(ln_elbow,) = ax_state.plot([], [], color='tab:orange', lw=1.3, label='elbow_state')
ln_near = ax_state.axhline(REACH_NEAR_M, color='gray', ls='--', lw=0.8)
ln_far  = ax_state.axhline(REACH_FAR_M,  color='gray', ls='--', lw=0.8)
ax_state.legend(loc='upper right', fontsize=7, ncol=2)
# Sits just above the axes rather than inside them: the reach curve runs high
# in the panel whenever the arm is extended, which is exactly when the badge
# matters most.
txt_level = ax_state.text(1.0, 1.04, '', transform=ax_state.transAxes,
                          va='bottom', ha='right', fontsize=7.5, family='monospace',
                          color='white',
                          bbox=dict(boxstyle='round,pad=0.25', facecolor='#555555',
                                    edgecolor='none'))

ax_log.axis('off')
ax_log.set_title('OSC log', loc='left', fontsize=8, pad=2)
txt_log  = ax_log.text(0.0, 1.0, '', transform=ax_log.transAxes,
                       va='top', ha='left', fontsize=7, family='monospace')
txt_rate = fig.text(0.99, 0.005, 'Rate: -- Hz', ha='right', va='bottom', fontsize=9)

# -- record button ------------------------------------------------------------
ax_btn  = fig.add_axes([0.03, 0.01, 0.09, 0.065])
btn_rec = Button(ax_btn, 'REC', color='#1a1a2e', hovercolor='#3a0000')
btn_rec.label.set_color('white')
btn_rec.label.set_fontsize(9)


def _toggle_record(_event=None):
    if not _recording:
        stem = f'osc_{time.strftime("%Y%m%d_%H%M%S")}'
        jsonl_p, csv_p, txt_p = _open_recording(stem)
        print(f'Recording -> {jsonl_p.name}  +  {csv_p.name}  +  {txt_p.name}')
        btn_rec.label.set_text('STOP')
        btn_rec.color      = '#550000'
        btn_rec.hovercolor = '#880000'
    else:
        _close_recording()
        btn_rec.label.set_text('REC')
        btn_rec.color      = '#1a1a2e'
        btn_rec.hovercolor = '#3a0000'


btn_rec.on_clicked(_toggle_record)


# -- helpers ------------------------------------------------------------------

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


def _padded_ylim(vals, minimum=0.1):
    """Tight limits with a 10% margin -- for data that isn't centred on zero."""
    lo, hi = min(vals), max(vals)
    pad = max((hi - lo) * 0.1, minimum)
    return lo - pad, hi + pad


def _set_reach_thresholds(near: float, far: float):
    """Move the dashed step lines, after --reach-near/--reach-far are parsed."""
    global REACH_NEAR_M, REACH_FAR_M
    REACH_NEAR_M, REACH_FAR_M = float(near), float(far)
    ln_near.set_ydata([REACH_NEAR_M, REACH_NEAR_M])
    ln_far.set_ydata([REACH_FAR_M, REACH_FAR_M])


# -- animation ----------------------------------------------------------------

def _animate(_frame):
    with _lock:
        pt = list(_pos_t); px = list(_pos_x)
        py = list(_pos_y); pz = list(_pos_z)
        vt = list(_vel_t); vx = list(_vel_x)
        vy = list(_vel_y); vz = list(_vel_z)
        at = list(_acc_t); axl = list(_acc_x)
        ayl = list(_acc_y); azl = list(_acc_z)
        rt = list(_reach_t); rv = list(_reach_v)
        et = list(_elbow_t); ev = list(_elbow_v)
        level = _level
        log = list(_log); hz = list(_hz_t)

    now = time.time()
    cut = -HISTORY_SECS

    # 3-D fading position trail
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

    # velocity x y z
    ax_vel.set_xlim(cut, 0)
    if vt:
        res = _window(vt, vx, vy, vz, now=now, cutoff=cut)
        tr_v, vxw, vyw, vzw = res[0], res[1], res[2], res[3]
        if tr_v:
            ln_vx.set_data(tr_v, vxw)
            ln_vy.set_data(tr_v, vyw)
            ln_vz.set_data(tr_v, vzw)
            ax_vel.set_ylim(*_symmetric_ylim(vxw + vyw + vzw))

    # acceleration x y z
    ax_acc.set_xlim(cut, 0)
    if at:
        res = _window(at, axl, ayl, azl, now=now, cutoff=cut)
        tr_a, axw, ayw, azw = res[0], res[1], res[2], res[3]
        if tr_a:
            ln_ax.set_data(tr_a, axw)
            ln_ay.set_data(tr_a, ayw)
            ln_az.set_data(tr_a, azw)
            ax_acc.set_ylim(*_symmetric_ylim(axw + ayw + azw))

    # arm state: reach + elbow_state, and the discrete level as a badge
    ax_state.set_xlim(cut, 0)
    tr_r, rw = _window(rt, rv, now=now, cutoff=cut)
    tr_e, ew = _window(et, ev, now=now, cutoff=cut)
    ln_reach.set_data(tr_r, rw)
    ln_elbow.set_data(tr_e, ew)
    if rw or ew:
        # The thresholds stay in view even when the arm is nowhere near them,
        # so the dashed lines are never off-screen and unexplained.
        ax_state.set_ylim(*_padded_ylim(list(rw) + list(ew)
                                        + [REACH_NEAR_M, REACH_FAR_M]))

    if rw or level is not None:
        reach_str = f'reach {rw[-1]:.3f} m' if rw else 'reach --'
        lvl_str = '-' if level is None else str(level)
        txt_level.set_text(f'{reach_str}   /reach/discrete = {lvl_str}')
        txt_level.get_bbox_patch().set_facecolor(
            '#555555' if level is None else LEVEL_COLORS[max(0, min(2, level))])
    else:
        txt_level.set_text('waiting for /rangen/reach  (state_interpreter running?)')
        txt_level.get_bbox_patch().set_facecolor('#555555')

    # rate + log
    rate_str = f'Rate: {sum(1 for t in hz if now - t < 1.0)} Hz'
    if _record_fh is not None:
        elapsed = now - (_record_start or now)
        m, s = divmod(int(elapsed), 60)
        rate_str += f'   REC  {m:02d}:{s:02d}'
    txt_rate.set_text(rate_str)
    txt_log.set_text('\n'.join(log[-12:]))


# -- OSC server ---------------------------------------------------------------

def _start_server(ip: str, port: int):
    d = osc_dispatcher.Dispatcher()
    d.map('/rangen/ee/pos',       _handle_pos)
    d.map('/rangen/ee/vel_lin',   _handle_vel_lin)
    d.map('/rangen/ee/accel_lin', _handle_accel_lin)
    # state_interpreter, forwarded by the bridge's generic signal layer.
    # '/rangen/reach' does not match '/rangen/reach/discrete' -- OSC address
    # matching is per-container, so these are three distinct handlers.
    d.map('/rangen/elbow_state',   _handle_elbow_state)
    d.map('/rangen/reach',         _handle_reach)
    d.map('/rangen/reach/discrete', _handle_reach_discrete)
    d.set_default_handler(_handle_generic)

    server = ThreadingOSCUDPServer((ip, port), d)
    server.allow_reuse_address = True
    print(f'OSC visualizer listening on {ip}:{port}')
    server.serve_forever()


# -- entry point --------------------------------------------------------------

def main():
    global TRAIL_SECS
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port',   type=int,   default=9004,
                        help='UDP port (default: 9004)')
    parser.add_argument('--ip',                 default='127.0.0.1',
                        help='Bind address (default: 127.0.0.1)')
    parser.add_argument('--trail',  type=float, default=TRAIL_SECS,
                        help=f'3-D trail length in seconds (default: {TRAIL_SECS})')
    parser.add_argument('--reach-near', type=float, default=REACH_NEAR_M,
                        help=f'where /rangen/reach/discrete steps 0->1, for the dashed '
                             f'line on the state panel (default: {REACH_NEAR_M}; must '
                             f'match reach_near_m in state_interpreter.yaml)')
    parser.add_argument('--reach-far', type=float, default=REACH_FAR_M,
                        help=f'where /rangen/reach/discrete steps 1->2 (default: '
                             f'{REACH_FAR_M}; must match reach_far_m)')
    parser.add_argument('--record', metavar='STEM', default=None,
                        help='Start recording immediately with this filename stem '
                             '(e.g. take01 -> logs/take01.csv + logs/take01.jsonl). '
                             'The REC button does the same with an auto timestamp.')
    args = parser.parse_args()
    TRAIL_SECS = args.trail
    _set_reach_thresholds(args.reach_near, args.reach_far)

    LOG_DIR.mkdir(parents=True, exist_ok=True)

    if args.record:
        jsonl_p, csv_p, txt_p = _open_recording(args.record)
        btn_rec.label.set_text('STOP')
        btn_rec.color = '#550000'
        print(f'Recording -> {jsonl_p}  +  {csv_p}  +  {txt_p}  (click STOP or close window)')

    threading.Thread(target=_start_server, args=(args.ip, args.port), daemon=True).start()

    ani = animation.FuncAnimation(  # noqa: F841
        fig, _animate, interval=80, blit=False, cache_frame_data=False,
    )

    # Ctrl+C alone doesn't reach us here: plt.show() blocks inside Tkinter's
    # C-level mainloop, which only hands control back to Python on a GUI
    # event, so a pending SIGINT can sit unprocessed indefinitely. A no-op
    # Tk timer forces a return to Python often enough to notice it.
    signal.signal(signal.SIGINT, lambda *_: plt.close('all'))
    tk_window = getattr(fig.canvas.manager, 'window', None)
    if tk_window is not None and hasattr(tk_window, 'after'):
        def _pump():
            tk_window.after(200, _pump)
        tk_window.after(200, _pump)

    try:
        plt.show()
    finally:
        if _recording:
            _close_recording()


if __name__ == '__main__':
    main()
