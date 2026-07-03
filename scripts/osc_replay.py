#!/usr/bin/env python3
"""
Replay a JSONL OSC recording at original timing -- use with Max/MSP or any
OSC receiver.

Format: JSONL produced by osc_visualizer.py (REC button or --record flag).
Each line: {"t": <unix_timestamp>, "addr": "/rangen/ee/pos", "args": [x, y, z]}

Max/MSP setup -- receives exactly the same OSC messages as the live bridge:
  [udpreceive 9004]
        |
  [oscparse]
        |
  [route /rangen/ee/pos /rangen/ee/vel_lin /rangen/ee/accel_lin]

Usage:
  python3 osc_replay.py logs/osc_20260702_143000.jsonl
  python3 osc_replay.py logs/osc_20260702_143000.jsonl --host 192.168.1.50
  python3 osc_replay.py logs/osc_20260702_143000.jsonl --speed 0.5   # half speed
  python3 osc_replay.py logs/osc_20260702_143000.jsonl --loop         # loop forever
  python3 osc_replay.py --list                                         # show recordings

Dependencies:
  pip install python-osc
"""

import argparse
import json
import pathlib
import sys
import time

try:
    from pythonosc import udp_client
except ImportError:
    sys.exit("python-osc not found.  Run: pip install python-osc")

_LOG_DIR = pathlib.Path(__file__).parent.parent / 'logs'


def _list_recordings():
    if not _LOG_DIR.exists():
        print(f'No recordings yet (logs/ folder does not exist: {_LOG_DIR})')
        return
    files = sorted(_LOG_DIR.glob('osc_*.jsonl'))
    if not files:
        print(f'No recordings in {_LOG_DIR}')
        return
    print(f'Recordings in {_LOG_DIR}:')
    for f in files:
        size_kb = f.stat().st_size / 1024
        n = sum(1 for _ in f.open())
        csv_p = f.with_suffix('.csv')
        csv_tag = f'  + {csv_p.name}' if csv_p.exists() else ''
        print(f'  {f.name}  ({n} events, {size_kb:.1f} kB){csv_tag}')


def replay(path: pathlib.Path, host: str, port: int, speed: float, loop: bool):
    client = udp_client.SimpleUDPClient(host, port)
    run = 0
    while True:
        events = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    continue

        if not events:
            sys.exit(f'No events in {path}')

        duration = events[-1]['t'] - events[0]['t']
        run += 1
        loop_tag = f' (loop {run})' if loop else ''
        print(f'Playing {len(events)} events [{duration:.1f}s @ {speed}x] -> {host}:{port}{loop_tag}')

        t0_file = events[0]['t']
        t0_wall = time.time()

        for msg in events:
            target = t0_wall + (msg['t'] - t0_file) / speed
            slack  = target - time.time()
            if slack > 0:
                time.sleep(slack)
            client.send_message(msg['addr'], msg['args'])

        print('Done.')
        if not loop:
            break
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('input', nargs='?', type=pathlib.Path, default=None,
                        help='JSONL recording from osc_visualizer.py')
    parser.add_argument('--host',  default='127.0.0.1',
                        help='Target host (default: 127.0.0.1)')
    parser.add_argument('--port',  type=int, default=9004,
                        help='Target UDP port (default: 9004)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed multiplier (default: 1.0)')
    parser.add_argument('--loop',  action='store_true',
                        help='Loop the recording indefinitely')
    parser.add_argument('--list',  action='store_true',
                        help='List available recordings in logs/ and exit')
    args = parser.parse_args()

    if args.list:
        _list_recordings()
        return

    if args.input is None:
        parser.error('input file required (or use --list)')

    if not args.input.exists():
        sys.exit(f'File not found: {args.input}')

    replay(args.input, args.host, args.port, args.speed, args.loop)


if __name__ == '__main__':
    main()
