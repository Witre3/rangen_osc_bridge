#!/usr/bin/env python3
"""
mcap_osc_player — replay a recorded MCAP bag as OSC and as a live Foxglove feed,
with no ROS2 anywhere.

Why this exists
---------------
The composer works on a Mac.  ROS2 Jazzy on macOS is a source build, so
ee_osc_bridge can't run there, and `ros2 bag play` can't either.  But nothing
about the bridge actually needs ROS: it reads message fields and does its own
arithmetic, and an MCAP file carries its message schemas inline.  So this script
reads the bag directly, replays it in wall-clock time, and drives both outputs
from one clock:

  * OSC/UDP  — the same addresses ee_osc_bridge sends, via the same
    ee_kinematics.emit_builtin() and the same osc_signals.yaml.
  * Foxglove — a WebSocket server the browser connects to, forwarding every
    channel's raw CDR bytes plus its inline schema, so even custom types
    (ranger_msgs, vision_msgs) render on a machine with no ROS2 installed.

One process owning both is the point: Foxglove's local-file mode has a seek bar
but no way to tell an external OSC player where it is, so the two would drift.
Here the sound and the picture come off the same message stream and cannot.

Usage
-----
    python3 -m rangen_osc_bridge.mcap_osc_player <bag.mcap | bag_dir>

Then open Chrome (not Safari or Firefox — they block ws:// from an https page)
at:
    https://app.foxglove.dev/~/view?ds=foxglove-websocket&ds.url=ws://localhost:8765

Adding more signals needs no code change here: add a row to the `generic:` list
in config/osc_signals.yaml and it is picked up on both sides.
"""

import argparse
import glob
import os
import pathlib
import sys
import threading
import time

# Imported as a package module in the repo, and as a flat module inside the
# macOS bundle (where app/ holds these files side by side and there is no
# package to be relative to).
try:
    from .ee_kinematics import EeState, OscSender, emit_builtin, pose_from_msg
    from .mcap_sources import McapSourceManager
    from .norm_curve import NormCurve, load_curves
    from .osc_signals import BUILTIN_SIGNALS, SignalConfig, load_signal_config
except ImportError:                                     # flat layout
    from ee_kinematics import EeState, OscSender, emit_builtin, pose_from_msg
    from mcap_sources import McapSourceManager
    from norm_curve import NormCurve, load_curves
    from osc_signals import BUILTIN_SIGNALS, SignalConfig, load_signal_config


FOXGLOVE_URL = ('https://app.foxglove.dev/~/view?ds=foxglove-websocket'
                '&ds.url=ws://localhost:{port}')

# Re-sent periodically during playback.  These are published once at t=0, so a
# viewer that connects after playback started would otherwise show no robot at
# all -- no TF tree and no model.
LATCHED_TOPICS = ('/tf_static', '/robot_description')


def _default_config(filename: str) -> str:
    """config/<filename>, wherever this copy of the package happens to live.

    Covers the repo layout (rangen_osc_bridge/../config), the macOS bundle
    layout (app/../config), and a colcon install (ament share dir).
    """
    here = pathlib.Path(__file__).resolve().parent
    for cand in (here.parent / 'config' / filename, here / 'config' / filename):
        if cand.exists():
            return str(cand)
    try:
        from ament_index_python.packages import get_package_share_directory
        p = pathlib.Path(get_package_share_directory('rangen_osc_bridge')) / 'config' / filename
        if p.exists():
            return str(p)
    except Exception:
        pass
    return ''


def open_foxglove(ws_port: int, warn) -> None:
    """Open the Foxglove deep link, preferring Chrome.

    Chrome is not a style preference: it is the only mainstream browser that
    allows a ws://localhost connection from the https Foxglove app.  Safari and
    Firefox block it as mixed content and show an empty, silently broken page.
    """
    import subprocess

    url = FOXGLOVE_URL.format(port=ws_port)
    if sys.platform == 'darwin':
        for browser in ('Google Chrome', 'Chromium', 'Brave Browser', 'Microsoft Edge'):
            if subprocess.run(['open', '-a', browser, url],
                              capture_output=True).returncode == 0:
                print(f'            opened in {browser}')
                return
        warn('Google Chrome not found — opening your default browser, but Safari and '
             'Firefox block ws://localhost and will show an empty page. '
             'Install Chrome from google.com/chrome.')
        subprocess.run(['open', url], capture_output=True)
    else:
        subprocess.run(['xdg-open', url], capture_output=True)


def resolve_bag(path: str) -> str:
    """Accept a .mcap file or a rosbag2 directory containing one."""
    p = pathlib.Path(path).expanduser()
    if p.is_file():
        return str(p)
    if p.is_dir():
        found = sorted(glob.glob(str(p / '*.mcap')))
        if not found:
            raise SystemExit(f'no .mcap file inside {p}')
        if len(found) > 1:
            print(f'note: {len(found)} .mcap files in {p}, playing {os.path.basename(found[0])}')
        return found[0]
    raise SystemExit(f'no such bag: {path}')


class Playback:
    """Transport state, shared between the Foxglove app and the replay loop.

    Declaring Capability.PlaybackControl gets the seek bar back: the app sends
    play / pause / seek / speed, this object records them, and the replay loop
    obeys.  Crucially the loop stays the single clock, so scrubbing moves the
    picture and the OSC stream together -- the sync that made a live feed worth
    the trade in the first place is not given up to get the transport back.
    """

    def __init__(self, start_ns: int, end_ns: int):
        self.start_ns = start_ns
        self.end_ns = end_ns
        self.lock = threading.Lock()
        self.wake = threading.Event()

        self.paused = False
        self.speed = 1.0
        self.current_ns = start_ns
        self.seek_to = None            # pending seek target, ns
        self.did_seek = False
        self.request_id = None
        self.ended = False

    def on_request(self, req) -> None:
        from foxglove.websocket import PlaybackCommand

        with self.lock:
            if getattr(req, 'seek_time', None) is not None:
                self.seek_to = int(req.seek_time)
                self.did_seek = True
                self.ended = False
            speed = getattr(req, 'playback_speed', None)
            if speed:
                self.speed = max(0.01, float(speed))
            cmd = getattr(req, 'playback_command', None)
            if cmd == PlaybackCommand.Play:
                self.paused = False
                self.ended = False
            elif cmd == PlaybackCommand.Pause:
                self.paused = True
            self.request_id = getattr(req, 'request_id', None)
        self.wake.set()

    def take_seek(self):
        with self.lock:
            s, self.seek_to = self.seek_to, None
            return s

    def snapshot(self):
        with self.lock:
            return self.paused, self.speed, self.current_ns

    def state(self):
        """A PlaybackState for broadcast_playback_state(), consuming did_seek."""
        from foxglove.websocket import PlaybackState, PlaybackStatus

        with self.lock:
            if self.ended:
                status = PlaybackStatus.Ended
            elif self.paused:
                status = PlaybackStatus.Paused
            else:
                status = PlaybackStatus.Playing
            st = PlaybackState(status, self.current_ns, self.speed,
                               self.did_seek, self.request_id)
            self.did_seek = False
            self.request_id = None
            return st


class _Listener:
    """Foxglove server callbacks: viewer-connected detection and the transport.

    The connected event holds playback until the browser is actually watching --
    the launcher opens Chrome a second or two after starting this process, and
    without the wait the opening seconds of the take (and its sound) are gone
    before anyone sees them.
    """

    def __init__(self, playback: 'Playback | None' = None):
        self.connected = threading.Event()
        self.playback = playback

    def on_subscribe(self, client, channel):
        self.connected.set()

    def on_playback_control_request(self, client, request):
        if self.playback is not None:
            self.playback.on_request(request)

    def __getattr__(self, _name):
        # ServerListener is a Protocol; ignore every other callback.
        return lambda *a, **k: None


def load_config(args, warn):
    """Signal selection and normalisation curves, with the node's fallbacks."""
    curve_vel = NormCurve.linear(0.0, 0.5)
    curve_acc = NormCurve.linear(0.0, 2.0)
    if args.curves and os.path.exists(args.curves):
        try:
            curves = load_curves(args.curves)
            curve_vel = curves.get('vel_lin_mag', curve_vel)
            curve_acc = curves.get('accel_lin_mag', curve_acc)
        except Exception as exc:
            warn(f'norm_curves.yaml unreadable ({exc}) — using linear defaults')

    # Missing / empty / malformed resolves to "all 17 built-ins, no generic
    # signals", exactly as in the node.
    cfg = SignalConfig.all_on()
    if args.signals and os.path.exists(args.signals):
        try:
            cfg = load_signal_config(args.signals)
        except Exception as exc:
            warn(f'osc_signals.yaml unreadable ({exc}) — sending all built-in EE signals')
    for problem in cfg.problems:
        warn(f'osc_signals.yaml: {problem}')
    return cfg, curve_vel, curve_acc


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Replay an MCAP bag as OSC + a live Foxglove feed (no ROS2 needed).')
    ap.add_argument('bag', help='.mcap file, or a rosbag2 directory containing one')
    ap.add_argument('--osc-ip', default='127.0.0.1')
    ap.add_argument('--osc-port', type=int, default=9000)
    ap.add_argument('--osc-ip-2', default='', help='optional second OSC target')
    ap.add_argument('--osc-port-2', type=int, default=9000)
    ap.add_argument('--rate', type=float, default=50.0, help='OSC send rate (Hz)')
    ap.add_argument('--smoothing-alpha', type=float, default=0.15)
    ap.add_argument('--pose-topic', default='/ee_pose_ref')
    ap.add_argument('--actual-pose-topic', default='/ee_pose_actual')
    ap.add_argument('--ws-host', default='127.0.0.1')
    ap.add_argument('--ws-port', type=int, default=8765)
    ap.add_argument('--topics', nargs='*', default=None,
                    help='only forward these topics to Foxglove (default: all)')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--loop', action='store_true')
    ap.add_argument('--wait-for-viewer', type=float, default=30.0,
                    help='seconds to wait for Foxglove to connect before starting '
                         '(0 starts immediately)')
    ap.add_argument('--signals', default=_default_config('osc_signals.yaml'))
    ap.add_argument('--curves', default=_default_config('norm_curves.yaml'))
    ap.add_argument('--generic-max-signals', type=int, default=32)
    ap.add_argument('--no-foxglove', action='store_true')
    ap.add_argument('--no-osc', action='store_true')
    ap.add_argument('--open-browser', action='store_true',
                    help='open the Foxglove deep link in Chrome once the server is up')
    args = ap.parse_args(argv)

    # The collaborator watches this scroll past in Terminal.  Without line
    # buffering the status lines sit in a pipe buffer and the window looks dead.
    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except Exception:
        pass

    def warn(msg):
        print(f'  ! {msg}', file=sys.stderr)

    bag = resolve_bag(args.bag)
    cfg, curve_vel, curve_acc = load_config(args, warn)

    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    fh = open(bag, 'rb')
    reader = make_reader(fh)
    summary = reader.get_summary()
    if summary is None:
        raise SystemExit(f'{bag} has no summary section — not a readable MCAP bag')

    # {topic: (channel, schema)} for every channel that has a CDR schema.
    channels = {}
    for chan in summary.channels.values():
        schema = summary.schemas.get(chan.schema_id)
        if schema is not None:
            channels[chan.topic] = (chan, schema)

    duration = 0.0
    if summary.statistics is not None and summary.statistics.message_count:
        duration = (summary.statistics.message_end_time
                    - summary.statistics.message_start_time) / 1e9

    print(f'bag       : {bag}')
    print(f'            {len(channels)} topics, {duration:.1f} s')

    # ── OSC ──────────────────────────────────────────────────────────────────
    targets = []
    if not args.no_osc:
        targets.append((args.osc_ip, args.osc_port))
        if args.osc_ip_2:
            targets.append((args.osc_ip_2, args.osc_port_2))
    sender = OscSender(targets, warn=warn) if targets else None

    ref = EeState(args.smoothing_alpha)
    act = EeState(args.smoothing_alpha)
    enabled_addrs = cfg.enabled_addresses()

    generic = McapSourceManager(warn=warn, max_signals=args.generic_max_signals)
    generic.bind(cfg.generic, channels)

    print(f'OSC       : {sender.describe() if sender else "disabled"}'
          f'   {len(enabled_addrs)}/{len(BUILTIN_SIGNALS)} built-in EE, {generic.status()}')

    # Only these get CDR-decoded; everything else is forwarded as raw bytes.
    decode_topics = set(generic.topics())
    for t, which in ((args.pose_topic, 'commanded'), (args.actual_pose_topic, 'actual')):
        if t in channels:
            decode_topics.add(t)
        else:
            warn(f'{t} is not in this bag — the {which} EE addresses stay at zero '
                 f'(older takes predate /ee_pose_actual)')

    factory = DecoderFactory()
    decoders = {t: factory.decoder_for('cdr', channels[t][1]) for t in decode_topics}

    # ── Foxglove ─────────────────────────────────────────────────────────────
    start_ns = end_ns = 0
    if summary.statistics is not None:
        start_ns = summary.statistics.message_start_time
        end_ns = summary.statistics.message_end_time
    playback = Playback(start_ns, end_ns)

    server = None
    listener = _Listener(playback)
    fg_channels = {}
    if not args.no_foxglove:
        import foxglove
        from foxglove import Capability

        foxglove.set_log_level('WARN')
        wanted = set(args.topics) if args.topics else set(channels)
        for topic, (chan, schema) in channels.items():
            if topic not in wanted:
                continue
            try:
                fg_channels[topic] = foxglove.Channel(
                    topic,
                    schema=foxglove.Schema(name=schema.name, encoding=schema.encoding,
                                           data=schema.data),
                    message_encoding=chan.message_encoding,
                )
            except Exception as exc:
                warn(f'cannot advertise {topic} ({schema.name}): {exc}')
        server = foxglove.start_server(
            name='rangen_mcap_player', host=args.ws_host, port=args.ws_port,
            server_listener=listener,
            capabilities=[Capability.PlaybackControl, Capability.Time],
            playback_time_range=(start_ns, end_ns) if end_ns > start_ns else None,
        )
        print(f'Foxglove  : ws://{args.ws_host}:{args.ws_port}   '
              f'{len(fg_channels)} topics advertised, play/pause/seek enabled')
        print(f'            {FOXGLOVE_URL.format(port=args.ws_port)}')
        if args.open_browser:
            open_foxglove(args.ws_port, warn)

    # ── the fixed-rate OSC thread ────────────────────────────────────────────
    # Mirrors the node's 50 Hz timer: the bridge emits at a constant rate no
    # matter how fast poses arrive, and the Max patch is built around that.
    lock = threading.Lock()
    stop = threading.Event()

    def osc_loop():
        period = 1.0 / args.rate
        next_t = time.perf_counter()
        while not stop.is_set():
            next_t += period
            with lock:
                emit_builtin(ref, act, enabled_addrs, curve_vel, curve_acc, sender.send)
                generic.emit(lambda addr, vals: sender.send(addr, [float(v) for v in vals]))
            time.sleep(max(0.0, next_t - time.perf_counter()))

    osc_thread = None
    if sender is not None:
        osc_thread = threading.Thread(target=osc_loop, name='osc', daemon=True)

    # Tell the app where the playhead is, so its seek bar tracks playback.
    def state_loop():
        while not stop.is_set():
            try:
                server.broadcast_playback_state(playback.state())
                server.broadcast_time(playback.current_ns)
            except Exception:
                pass
            stop.wait(0.1)

    state_thread = (threading.Thread(target=state_loop, name='playback-state', daemon=True)
                    if server is not None else None)

    if server is not None and args.wait_for_viewer > 0:
        print(f'\nwaiting up to {args.wait_for_viewer:.0f}s for Foxglove to connect '
              f'(Ctrl-C to stop)...')
        if listener.connected.wait(args.wait_for_viewer):
            print('Foxglove connected.')
            time.sleep(0.5)          # let the rest of its subscriptions land
        else:
            print('no viewer yet — starting anyway.')

    if osc_thread is not None:
        osc_thread.start()
    if state_thread is not None:
        state_thread.start()

    playback.speed = args.speed
    print(f'\nplaying at {args.speed:g}x — Ctrl-C to stop'
          f'{"" if server is None else "  (play/pause/seek from Foxglove)"}\n')

    def reset_states():
        """After a seek, derived velocity from a discontinuous jump is garbage."""
        with lock:
            ref.__init__(args.smoothing_alpha)
            act.__init__(args.smoothing_alpha)
            generic.reset()

    try:
        _play(reader, args, playback, fg_channels, decoders, decode_topics,
              ref, act, generic, lock, stop, duration, reset_states)
    except KeyboardInterrupt:
        print('\nstopped.')
    finally:
        stop.set()
        for th in (osc_thread, state_thread):
            if th is not None:
                th.join(timeout=1.0)
        if server is not None:
            try:
                server.stop()
            except Exception:
                pass
        fh.close()

    return 0


def _play(reader, args, playback, fg_channels, decoders, decode_topics,
          ref, act, generic, lock, stop, duration, reset_states):
    """Replay the bag, obeying pause / seek / speed from the Foxglove app.

    Structured as an outer restart loop around mcap's iter_messages(start_time=)
    so a seek is just "abandon this iterator and open a new one at the target".
    """
    latched = {}                    # topic -> raw payload, re-sent periodically
    cursor = playback.start_ns
    count = 0

    while not stop.is_set():
        # Wall-clock anchor for pacing.  Re-established after every pause, seek
        # or speed change, so the playhead never tries to "catch up" for time
        # spent paused.
        anchor_wall = time.perf_counter()
        anchor_log = cursor
        anchor_speed = playback.speed
        next_relatch = 0.0
        next_report = 0.0
        restart = False

        for _schema, channel, message in reader.iter_messages(start_time=cursor):
            if stop.is_set():
                return

            # ── transport ────────────────────────────────────────────────────
            paused, speed, _ = playback.snapshot()
            while paused and not stop.is_set():
                playback.wake.wait(0.1)
                playback.wake.clear()
                paused, speed, _ = playback.snapshot()
                if not paused:                      # re-anchor on resume
                    anchor_wall = time.perf_counter()
                    anchor_log = message.log_time
                    anchor_speed = speed
            if stop.is_set():
                return

            seek = playback.take_seek()
            if seek is not None:
                cursor = max(playback.start_ns, min(int(seek), playback.end_ns))
                playback.current_ns = cursor
                reset_states()
                restart = True
                break

            if speed != anchor_speed:                # re-anchor on speed change
                anchor_wall = time.perf_counter()
                anchor_log = message.log_time
                anchor_speed = speed

            target = anchor_wall + (message.log_time - anchor_log) / 1e9 / anchor_speed
            lag = target - time.perf_counter()
            if lag > 0:
                time.sleep(lag)

            playback.current_ns = message.log_time
            cursor = message.log_time + 1

            # ── forward ──────────────────────────────────────────────────────
            topic = channel.topic
            fg = fg_channels.get(topic)
            if fg is not None:
                try:
                    fg.log(message.data, log_time=message.log_time)
                except Exception:
                    pass            # a viewer disconnecting mid-send is not fatal
                if topic in LATCHED_TOPICS:
                    latched[topic] = message.data

            if topic in decode_topics:
                try:
                    msg = decoders[topic](message.data)
                except Exception:
                    msg = None
                if msg is not None:
                    with lock:
                        if topic == args.pose_topic:
                            ref.update(*pose_from_msg(msg))
                        elif topic == args.actual_pose_topic:
                            act.update(*pose_from_msg(msg))
                        generic.on_message(topic, msg)

            count += 1
            now_bag = (message.log_time - playback.start_ns) / 1e9

            if now_bag >= next_relatch:
                next_relatch = now_bag + 2.0
                for lt, payload in latched.items():
                    try:
                        fg_channels[lt].log(payload, log_time=message.log_time)
                    except Exception:
                        pass
            if now_bag >= next_report:
                next_report = now_bag + 5.0
                with lock:
                    p = ref.pos
                    v = ref.vel_lin_mag
                print(f'  t={now_bag:6.1f}/{duration:.0f}s  msgs={count:7d}  '
                      f'pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})  |vel|={v:.3f} m/s')

        if restart:
            continue                # seek: reopen the iterator at the new cursor

        # Reached the end of the file.
        if args.loop:
            print('\n--- loop ---\n')
            cursor = playback.start_ns
            reset_states()
            continue

        playback.ended = True
        print('\nend of bag — seek or press play in Foxglove to replay, Ctrl-C to quit')
        while not stop.is_set():
            playback.wake.wait(0.2)
            playback.wake.clear()
            seek = playback.take_seek()
            if seek is not None:
                cursor = max(playback.start_ns, min(int(seek), playback.end_ns))
                playback.current_ns = cursor
                playback.ended = False
                reset_states()
                break
            if not playback.paused and not playback.ended:
                cursor = playback.start_ns
                reset_states()
                break
        else:
            return


if __name__ == '__main__':
    sys.exit(main())
