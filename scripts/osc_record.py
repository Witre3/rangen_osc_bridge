#!/usr/bin/env python3
"""
Headless OSC recorder for ee_osc_bridge -- no matplotlib/Tk, safe to launch
as a background subprocess (used by record_demo.sh to capture the OSC
stream alongside a rosbag). For live visualization use osc_visualizer.py
instead -- both write the same three file formats.

  python3 osc_record.py --out /path/to/session_dir/session
  python3 osc_record.py --out /path/to/session_dir/session --ip 127.0.0.1 --port 9004

Writes three files next to the given stem, creating parent directories as
needed:
  <stem>.jsonl   one JSON object per message with the full arg list --
                 {"t": <unix time>, "addr": "...", "args": [...]} --
                 for osc_replay.py / osc_to_midi.py
  <stem>.csv     t,addr,arg0,arg1,arg2 for Excel/pandas -- matches
                 osc_visualizer.py's schema, so args beyond the first 3
                 are dropped here (full args are in the .jsonl)
  <stem>.txt     space-separated, for a Max text object + zl slice

Stop with SIGINT/SIGTERM -- files are flushed and closed on shutdown.
"""

import argparse
import json
import pathlib
import signal
import sys
import threading
import time

try:
    from pythonosc import dispatcher as osc_dispatcher
    from pythonosc.osc_server import ThreadingOSCUDPServer
except ImportError:
    sys.exit("python-osc not found.  Run: pip install python-osc")


_jsonl_fh = None
_csv_fh = None
_txt_fh = None
_lock = threading.Lock()


def _handle(addr, *args):
    numeric = [a for a in args if isinstance(a, (int, float))]
    now = time.time()
    padded = list(numeric) + ['', '']
    with _lock:
        _jsonl_fh.write(json.dumps({'t': now, 'addr': addr, 'args': numeric}) + '\n')
        _csv_fh.write(f'{now:.6f},{addr},{padded[0]},{padded[1]},{padded[2]}\n')
        _txt_fh.write(f'{now:.6f} {addr} {padded[0]} {padded[1]} {padded[2]}\n')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', required=True,
                        help='Output path stem, e.g. /path/to/session_dir/session '
                             '(writes <stem>.jsonl, <stem>.csv, <stem>.txt)')
    parser.add_argument('--ip', default='127.0.0.1', help='Bind address (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=9004, help='UDP port (default: 9004)')
    args = parser.parse_args()

    stem = pathlib.Path(args.out)
    stem.parent.mkdir(parents=True, exist_ok=True)
    jsonl_path = stem.parent / f'{stem.name}.jsonl'
    csv_path   = stem.parent / f'{stem.name}.csv'
    txt_path   = stem.parent / f'{stem.name}.txt'

    global _jsonl_fh, _csv_fh, _txt_fh
    _jsonl_fh = open(jsonl_path, 'w')
    _csv_fh   = open(csv_path, 'w')
    _txt_fh   = open(txt_path, 'w')
    _csv_fh.write('t,addr,arg0,arg1,arg2\n')

    d = osc_dispatcher.Dispatcher()
    d.set_default_handler(_handle)

    server = ThreadingOSCUDPServer((args.ip, args.port), d)
    server.allow_reuse_address = True

    def _shutdown(*_a):
        # Runs on the main thread; serve_forever() runs on server_thread, so
        # this does not deadlock the way calling shutdown() from the same
        # thread running serve_forever() would.
        server.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print(f'[osc_record] listening on {args.ip}:{args.port} -> {stem.parent}/{stem.name}.{{jsonl,csv,txt}}',
          file=sys.stderr)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    server_thread.join()

    for fh in (_jsonl_fh, _csv_fh, _txt_fh):
        fh.flush()
        fh.close()
    print(f'[osc_record] saved -> {jsonl_path.name}, {csv_path.name}, {txt_path.name}  in {stem.parent}',
          file=sys.stderr)


if __name__ == '__main__':
    main()
