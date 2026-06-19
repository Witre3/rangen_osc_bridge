#!/usr/bin/env python3
"""
Convert an OSC recording (JSONL from osc_visualizer.py --record) to a MIDI
file that Ableton Live can import as CC automation.

Usage:
  python3 osc_to_midi.py take01.jsonl                      # uses default config
  python3 osc_to_midi.py take01.jsonl -o session1.mid
  python3 osc_to_midi.py take01.jsonl -c my_mapping.yaml
  python3 osc_to_midi.py take01.jsonl --thin 20            # keep 1 in 20 events

The output is a Type-0 MIDI file (single track).  Drag it into an Ableton
MIDI track, then use "Unfold" to see each CC lane as an automation clip.
Map CC numbers to instrument parameters with MIDI Learn (Cmd/Ctrl+M in Live).

Config file format: see config/osc_to_midi.yaml.

Dependencies:
  pip install mido pyyaml
"""

import argparse
import json
import pathlib
import sys

try:
    import yaml
except ImportError:
    sys.exit("PyYAML not found.  Run: pip install pyyaml")

try:
    import mido
except ImportError:
    sys.exit("mido not found.  Run: pip install mido")


_DEFAULT_CONFIG = pathlib.Path(__file__).parent.parent / 'config' / 'osc_to_midi.yaml'


def _load_config(path: pathlib.Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def _scale(value: float, v_min: float, v_max: float) -> int:
    """Map a float in [v_min, v_max] to a MIDI CC integer in [0, 127]."""
    span = v_max - v_min
    if span == 0:
        return 64
    normalized = (value - v_min) / span
    return max(0, min(127, int(normalized * 127)))


def convert(
    jsonl_path: pathlib.Path,
    out_path: pathlib.Path,
    config: dict,
    thin: int = 1,
) -> None:
    midi_cfg = config.get('midi', {})
    bpm      = float(midi_cfg.get('bpm', 120))
    ppq      = int(midi_cfg.get('ppq', 480))
    channel  = int(midi_cfg.get('channel', 1)) - 1  # convert 1-16 → 0-15

    ticks_per_sec = ppq / (60.0 / bpm)

    # Build a lookup: addr -> list of (arg_index, cc, v_min, v_max, label)
    mapping: dict[str, list] = {}
    for entry in config.get('mappings', []):
        addr = entry['addr']
        mapping.setdefault(addr, []).append((
            int(entry['arg']),
            int(entry['cc']),
            float(entry['range'][0]),
            float(entry['range'][1]),
            entry.get('label', f"cc{entry['cc']}"),
        ))

    # Read JSONL and collect raw events
    # events: list of (abs_tick, cc, value_int, label)
    events: list[tuple[int, int, int, str]] = []
    t0 = None
    n_lines = 0
    skipped = 0

    with open(jsonl_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                skipped += 1
                continue

            addr = msg['addr']
            if addr not in mapping:
                continue

            t = float(msg['t'])
            if t0 is None:
                t0 = t
            abs_tick = int((t - t0) * ticks_per_sec)

            for (arg_idx, cc, v_min, v_max, label) in mapping[addr]:
                args = msg.get('args', [])
                if arg_idx >= len(args):
                    continue
                cc_val = _scale(float(args[arg_idx]), v_min, v_max)
                events.append((abs_tick, cc, cc_val, label))
            n_lines += 1

    if not events:
        sys.exit(
            f'No mapped events found in {jsonl_path}.\n'
            f'Check that OSC addresses in the config match what the bridge sends.'
        )

    # Apply thinning (keep every Nth event per CC channel)
    if thin > 1:
        per_cc: dict[int, list] = {}
        for ev in events:
            per_cc.setdefault(ev[1], []).append(ev)
        events = []
        for cc_evs in per_cc.values():
            events.extend(cc_evs[::thin])

    events.sort(key=lambda e: e[0])

    # Build MIDI file
    mid = mido.MidiFile(type=0, ticks_per_beat=ppq)
    track = mido.MidiTrack()
    mid.tracks.append(track)

    # Set tempo
    track.append(mido.MetaMessage(
        'set_tempo', tempo=mido.bpm2tempo(bpm), time=0
    ))

    # Write CC events as delta-tick messages
    prev_tick = 0
    for abs_tick, cc, value, _label in events:
        delta = abs_tick - prev_tick
        track.append(mido.Message(
            'control_change',
            channel=channel,
            control=cc,
            value=value,
            time=delta,
        ))
        prev_tick = abs_tick

    mid.save(str(out_path))

    # Summary
    duration_s = (prev_tick / ticks_per_sec) if ticks_per_sec else 0
    cc_used = sorted({e[1] for e in events})
    labels = {
        entry['cc']: entry.get('label', f"cc{entry['cc']}")
        for entry in config.get('mappings', [])
    }

    print(f'\nConverted: {jsonl_path}  →  {out_path}')
    print(f'  Duration : {duration_s:.1f} s  ({int(duration_s // 60):02d}:{int(duration_s % 60):02d})')
    print(f'  Events   : {len(events)} CC messages  ({n_lines} source lines read)')
    if skipped:
        print(f'  Skipped  : {skipped} malformed lines')
    print(f'  BPM/PPQ  : {bpm:.0f} / {ppq}  (MIDI channel {channel + 1})')
    lane_strs = [f'CC{c} ({labels.get(c, "?")})' for c in cc_used]
    print(f'  CC lanes : {", ".join(lane_strs)}')
    print('\nIn Ableton: drag the .mid file onto a MIDI track, then use')
    print('MIDI Learn (Cmd+M / Ctrl+M) to map each CC to a synth parameter.\n')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('input',  type=pathlib.Path,
                        help='JSONL recording from osc_visualizer.py --record')
    parser.add_argument('-o', '--output', type=pathlib.Path, default=None,
                        help='Output .mid file (default: <input>.mid)')
    parser.add_argument('-c', '--config', type=pathlib.Path,
                        default=_DEFAULT_CONFIG,
                        help=f'YAML mapping config (default: {_DEFAULT_CONFIG})')
    parser.add_argument('--thin', type=int, default=1, metavar='N',
                        help='Keep 1 in every N events per CC lane to reduce '
                             'MIDI density (default: 1 = keep all)')
    args = parser.parse_args()

    if not args.input.exists():
        sys.exit(f'Input file not found: {args.input}')
    if not args.config.exists():
        sys.exit(f'Config file not found: {args.config}')

    out = args.output or args.input.with_suffix('.mid')
    cfg = _load_config(args.config)
    convert(args.input, out, cfg, thin=args.thin)


if __name__ == '__main__':
    main()
