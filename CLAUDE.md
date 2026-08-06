# CLAUDE.md — rangen_osc_bridge

**Setting this up on a new machine, or something host-side is missing?
Read `first_time_setup.md` first** — it records the Python dependencies,
network requirements and shell setup that live outside git.

## What this repo is

The OSC bridge for the Rangen × HoMeR project: turns robot state into OSC (UDP)
for music software (Ableton Live, Max/MSP). `README.md` is the full reference —
this file is the orientation an agent needs before touching anything.

**This package is NOT part of the haply-teleop container image.** That image
builds `src/ui` only. This one runs bare-metal or standalone, which is why its
dependencies are a separate concern (see `first_time_setup.md`).

## Two kinds of code here, and the split matters

| Kind | Files | Needs ROS? |
|---|---|---|
| ROS 2 nodes | `rangen_osc_bridge/ee_osc_bridge.py`, `state_interpreter.py` | yes |
| Standalone tools | `scripts/osc_visualizer.py`, `osc_record.py`, `curve_editor.py`, `osc_to_midi.py`, `mcap_osc_player.py` | **no** |

The standalone half is deliberate: a collaborator on a Mac with no ROS install
must be able to receive the stream and replay a recorded take. `mcap_osc_player.py`
reads an MCAP bag directly (schemas are inline in the file) and drives both OSC
and a Foxglove WebSocket off one clock. **Do not introduce a ROS import into
anything under `scripts/`** — that is the whole point of the split.

## Adding a signal

Signals are wired in `config/osc_signals.yaml`, not in code. A `generic:` row
maps a ROS topic + field to an OSC address and arity, and both the live bridge
and the MCAP player honour it with no code change. `state_interpreter`'s three
outputs are ordinary generic rows — the bridge has no special knowledge of them.

Their addresses are set **by hand** rather than auto-derived, because
`/rangen/reach/discrete` is what the composer's Max patch routes on; the
derived `/rangen/rangen/state/reach_discrete` is not. If a row is ever re-saved
from a GUI, keep those addresses.

## House rules

- **This repo has two remotes and both are kept in sync.** Push to both, or the
  one you skip goes quietly stale — GitLab was 12 commits behind before anyone
  noticed on 2026-08-06.

  | Remote | URL | Role |
  |---|---|---|
  | `github` | `git@github.com:Witre3/rangen_osc_bridge.git` | Tracked upstream of `main` |
  | `origin` | `git@git.initrobots.ca:rangen/internal/rangen_osc_bridge.git` | Team mirror; `rangen_ws/CLAUDE.md` points here |

  ```bash
  git push github main && git push origin main
  ```

  Note `origin` is **not** the tracked upstream here, so a bare `git push`
  goes to `github` only. `git log --oneline HEAD --not --remotes=origin`
  tells you what GitLab is missing.
- Branch is `main`.
- Some older commits carry a `test <test@localhost>` author. They are already
  published; leave them. Check `git config user.email` before committing.
- `osc_record.py` is invoked by `~/demonstrations/rosbag_recording_tools/scripts/record_demo.sh`
  by absolute path. Moving or renaming it breaks that script silently.
