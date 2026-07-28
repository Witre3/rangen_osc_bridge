#!/usr/bin/env python3
"""
mac_launcher — what "Start Rangen.command" runs on the collaborator's Mac.

Everything the non-technical path needs, in one place: find the takes, ask which
one, then hand over to mcap_osc_player.  Written in Python rather than in the
.command shell script so the same logic can be tested on Linux, and so the only
shell involved is four lines that pick an interpreter.

Bags are looked for in ./bags next to the app *and* in /Volumes/*/bags, because
the app is unzipped to the Desktop (exFAT cannot store the executable bit, so
unzipping on the stick would leave the launcher un-double-clickable) while the
takes stay on the USB stick where there is room for them.
"""

import glob
import os
import pathlib
import sys

try:
    from .mcap_osc_player import main as player_main
except ImportError:
    from mcap_osc_player import main as player_main


BANNER = r"""
  ____                             ____  _
 |  _ \ __ _ _ __   __ _  ___ _ __|  _ \| | __ _ _   _  ___ _ __
 | |_) / _` | '_ \ / _` |/ _ \ '_ \ |_) | |/ _` | | | |/ _ \ '__|
 |  _ < (_| | | | | (_| |  __/ | | |  __/| | (_| | |_| |  __/ |
 |_| \_\__,_|_| |_|\__, |\___|_| |_|_|   |_|\__,_|\__, |\___|_|
                   |___/                          |___/
   robot recording  ->  Foxglove (picture)  +  OSC (sound)
"""


def find_bags():
    """Every take we can see, as (label, path), nearest first."""
    roots = []
    here = pathlib.Path(__file__).resolve().parent.parent
    roots.append(here / 'bags')
    roots.extend(pathlib.Path(v) / 'bags' for v in glob.glob('/Volumes/*'))
    roots.append(pathlib.Path.home() / 'Desktop' / 'bags')

    found = []
    seen = set()
    for root in roots:
        if not root.is_dir():
            continue
        # A take is either a .mcap file or a rosbag2 directory holding one.
        for entry in sorted(root.iterdir()):
            if entry.name.startswith('.'):
                continue
            path = None
            if entry.is_file() and entry.suffix == '.mcap':
                path = entry
            elif entry.is_dir() and glob.glob(str(entry / '*.mcap')):
                path = entry
            if path is None:
                continue
            key = path.name
            if key in seen:
                continue
            seen.add(key)
            found.append((path.name, str(path)))
    return found


def choose(bags):
    if len(bags) == 1:
        print(f'Playing the only take found:\n  {bags[0][0]}\n')
        return bags[0][1]

    print('Which take do you want to play?\n')
    for i, (label, _p) in enumerate(bags, 1):
        print(f'  {i:2d}.  {label}')
    print()
    while True:
        try:
            raw = input(f'Type a number from 1 to {len(bags)} and press Return: ').strip()
        except (EOFError, KeyboardInterrupt):
            return None
        if raw.isdigit() and 1 <= int(raw) <= len(bags):
            return bags[int(raw) - 1][1]
        print('  Sorry, that is not one of the numbers above.')


def _hold():
    """Keep the Terminal window up so a message can actually be read.

    Double-clicked .command windows close on exit, taking any error with them.
    """
    if os.environ.get('RANGEN_HOLD_WINDOW'):
        try:
            input('\nPress Return to close this window. ')
        except (EOFError, KeyboardInterrupt):
            pass


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    print(BANNER)

    # A take dragged onto the launcher, or named on the command line, wins.
    # Only a leading positional counts -- anything after the first option is an
    # option's value, not a bag.
    if argv and not argv[0].startswith('-'):
        bag = argv.pop(0)
    else:
        bags = find_bags()
        if not bags:
            print('No recordings found.\n')
            print('Put the .mcap files in the "bags" folder on the USB stick,')
            print('or in a "bags" folder next to this app, then start it again.')
            _hold()
            return 1
        bag = choose(bags)
        if bag is None:
            return 1

    print()
    try:
        rc = player_main([bag, '--open-browser'] + argv)
    except SystemExit as exc:
        print(f'\n{exc}')
        _hold()
        return 1
    except Exception as exc:
        print(f'\nSomething went wrong: {exc.__class__.__name__}: {exc}')
        _hold()
        return 1
    _hold()
    return rc


if __name__ == '__main__':
    sys.exit(main())
