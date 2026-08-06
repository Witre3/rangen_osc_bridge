# rangen_osc_bridge

OSC bridge: publishes Rangen end-effector pose, velocity, and acceleration
over OSC (UDP) for mapping into music software (Ableton Live, Max/MSP, etc).
See `/home/wt/rangen_ws/CLAUDE.md` (Composer Bridge section) for how this
fits into the wider Rangen × HoMeR project.

## What's in this package

| File | Role |
|---|---|
| `rangen_osc_bridge/ee_osc_bridge.py` | ROS2 node — subscribes to EE pose, derives velocity/acceleration, sends OSC |
| `rangen_osc_bridge/state_interpreter.py` | ROS2 node — reads the arm's TF tree and publishes the derived state topics the bridge forwards as `/rangen/elbow_state`, `/rangen/reach`, `/rangen/reach/discrete` |
| `rangen_osc_bridge/norm_curve.py` | Shared breakpoint-curve normalization (0-1), used by both the bridge and the curve editor |
| `scripts/osc_visualizer.py` | Standalone (no ROS2) live monitor — 3-D trail + vel/accel plots + OSC log |
| `scripts/osc_record.py` | Standalone (no ROS2), headless — records OSC to JSONL/CSV/txt, no GUI; used by `~/demonstrations/scripts/record_demo.sh` to capture OSC alongside a rosbag |
| `scripts/curve_editor.py` | Standalone (no ROS2) GUI — drag/bend curve breakpoints for the `/mag/norm` outputs against live OSC data |
| `scripts/osc_to_midi.py` | Converts a recorded OSC session (JSONL) to a MIDI file for Ableton |
| `config/ee_osc_bridge.yaml` | OSC target IP/port, send rate, smoothing, pose topic, norm curve file path |
| `config/state_interpreter.yaml` | TF frames (base/elbow/EE), publish rate, and the reach thresholds |
| `config/osc_signals.yaml` | Which built-in EE addresses are sent, plus the generic topic→address rows (this is where `state_interpreter`'s three signals are wired to their OSC addresses) |
| `config/norm_curves.yaml` | Tunable 0-1 normalization curves for `vel_lin_mag` / `accel_lin_mag`, edited by `curve_editor.py`, hot-reloaded by the bridge |
| `config/osc_to_midi.yaml` | OSC address → MIDI CC mapping used by `osc_to_midi.py` |
| `launch/ee_osc_bridge.launch.py` | Launches `ee_osc_bridge` with the yaml config |

## One-time setup

Python dependencies the scripts need (ROS2 deps are handled by
`package.xml` / colcon):

```bash
pip install python-osc mido pyyaml matplotlib numpy
```

Build the package as part of the workspace:

```bash
cd /home/wt/rangen_ws
colcon build --symlink-install --packages-select rangen_osc_bridge
source install/setup.bash
```

## Launch the bridge node

Requires the teleop/UI stack running so `/ee_pose_ref` is being published
(see `/home/wt/rangen_ws/CLAUDE.md`).

```bash
source /home/wt/rangen_ws/install/setup.bash
ros2 launch rangen_osc_bridge ee_osc_bridge.launch.py
```

Default OSC target is `127.0.0.1:9000`. To send to another machine on the
network (e.g. the composer's laptop running Ableton), override at launch:

```bash
ros2 launch rangen_osc_bridge ee_osc_bridge.launch.py \
  osc_target_ip:=192.168.1.42 osc_target_port:=9001
```

Or edit `/home/wt/rangen_ws/src/rangen_osc_bridge/config/ee_osc_bridge.yaml`
directly (also has the smoothing and norm-curve-file settings). The actual
normalization curves live in a separate file — see "Tune normalization
curves" below.

This launch file also starts `state_interpreter` (next section). Leave it out
— e.g. when there is no TF tree to read — with:

```bash
ros2 launch rangen_osc_bridge ee_osc_bridge.launch.py state_interpreter:=false
```

## Arm state signals (state_interpreter)

`state_interpreter` reads the arm's TF tree and reduces it to three scalars,
published as ordinary ROS topics and forwarded to Max by the bridge:

| ROS topic | OSC address | args | Meaning |
|---|---|---|---|
| `/rangen/state/elbow_state` | `/rangen/elbow_state` | 1 | elbow z − EE z (m) — positive when the elbow rides above the hand |
| `/rangen/state/reach` | `/rangen/reach` | 1 | straight-line distance from the arm base to the EE (m) |
| `/rangen/state/reach_discrete` | `/rangen/reach/discrete` | 1 | `0` below 0.6 m, `2` above 0.95 m, `1` in between |

Both ends of `elbow_state` come from the same TF snapshot
(`gen3_base_link` → `gen3_forearm_link` and → `gen3_robotiq_85_tool_link`), so
the difference can never mix two poses sampled at different instants. `reach`
is the norm of the EE translation in `gen3_base_link`, i.e. the distance from
the arm base itself. Nothing is published until TF resolves — a missing
transform sends silence, not a zero.

The node never opens a UDP socket: the bridge does the sending, so all three
ride the same 50 Hz tick as the EE block and every OSC address/arity stays in
one file. The wiring is the three `generic:` rows in
`config/osc_signals.yaml` — edit an address there, not in the node.

Tunables live in `config/state_interpreter.yaml`: the three frame names, the
publish rate, `reach_near_m` / `reach_far_m`, and `reach_hysteresis_m` (0 by
default; set it to ~0.02 if a hand parked on a threshold makes the discrete
output chatter at the send rate).

Run it on its own — the bridge picks its topics up either way:

```bash
ros2 run rangen_osc_bridge state_interpreter --ros-args \
  --params-file /home/wt/rangen_ws/install/rangen_osc_bridge/share/rangen_osc_bridge/config/state_interpreter.yaml
```

## Launch the visualizer (no ROS2 required)

Run this on any machine that can receive the UDP OSC stream — the same
machine, or a laptop on the same network as `osc_target_ip`:

```bash
python3 /home/wt/rangen_ws/src/rangen_osc_bridge/scripts/osc_visualizer.py
```

Useful flags:

```bash
python3 scripts/osc_visualizer.py --port 9001           # match a non-default port
python3 scripts/osc_visualizer.py --trail 5              # shorter 3-D trail (seconds)
python3 scripts/osc_visualizer.py --record take01.jsonl  # record while visualising
python3 scripts/osc_visualizer.py --reach-near 0.5 --reach-far 1.0  # retuned thresholds
```

Panels, top to bottom on the right: linear velocity, linear acceleration, arm
state, and the OSC log. The arm-state panel plots `/rangen/reach` and
`/rangen/elbow_state` on one metre axis, with dashed lines where
`/rangen/reach/discrete` steps and a badge showing the current level —
green 0, amber 1, red 2. `--reach-near` / `--reach-far` only move the dashed
lines; the levels themselves come from `state_interpreter`, so keep the two in
sync with `config/state_interpreter.yaml` if you retune them.

## Tune normalization curves (curve_editor.py)

The bridge sends two pre-normalized 0-1 channels — `/rangen/ee/vel_lin/mag/norm`
and `/rangen/ee/accel_lin/mag/norm` — mapped through a tunable curve (not a
fixed linear min/max). The curve is a set of draggable breakpoints, each
with its own bend, like a DAW clip-fade handle: flat, ease-in, ease-out, or
anything in between. It's defined in
`/home/wt/rangen_ws/src/rangen_osc_bridge/config/norm_curves.yaml` and edited
with the GUI below. `ee_osc_bridge` polls that file's mtime and hot-reloads
it, so a saved change applies immediately with no node restart.

Run this on any machine that can receive the raw (unnormalized) magnitude
OSC messages the bridge sends — same machine, or the same laptop you'd run
the visualizer on:

```bash
python3 /home/wt/rangen_ws/src/rangen_osc_bridge/scripts/curve_editor.py
```

Useful flags:

```bash
python3 scripts/curve_editor.py --port 9003                # match a non-default port
python3 scripts/curve_editor.py -c my_norm_curves.yaml      # tune a different curve file
```

**Workflow:**

1. While tuning, point the bridge at the editor's port (default `9002`) so
   the editor's live playhead shows you where real motion actually falls on
   the curve:
   ```bash
   ros2 launch rangen_osc_bridge ee_osc_bridge.launch.py \
     osc_target_port:=9002
   ```
2. In the GUI window (one panel per channel — `vel_lin_mag`, `accel_lin_mag`):
   - **drag a point** to move it (the two endpoints are locked to the domain
     edges, x=0 and x=input_max, but their y value is still draggable)
   - **drag a segment's curve line** (grab it near its midpoint, shown as a
     small orange dot) to bend that segment — up for ease-in, down for
     ease-out
   - **double-click empty space** to add a new breakpoint
   - **right-click a point** to delete it (endpoints can't be deleted)
   - press **`s`** to save both curves to `norm_curves.yaml`
   - press **`r`** to reset both curves back to linear
3. Once you're happy, point `osc_target_port` back at Ableton/the
   visualizer (`9000`, or whatever your session uses) — the saved curve
   keeps applying regardless of who's listening.

Note: only one process can receive the bridge's OSC stream at a time
(unicast UDP, not multicast) — you can't watch the visualizer and tune the
curve editor simultaneously without pointing the bridge back and forth.

## Record OSC headlessly (no GUI)

`scripts/osc_record.py` is a stripped-down version of the visualizer with
no matplotlib/Tk — it just opens the UDP socket and writes the same three
formats the visualizer's REC button does, so it's safe to launch as a
background subprocess (no display required, e.g. over SSH). It listens on
the same secondary OSC target the visualizer uses by default
(`127.0.0.1:9004`, see `osc_target_ip_2`/`osc_target_port_2` in
`ee_osc_bridge.yaml`), so it competes with the visualizer for that port —
don't run both at once against the same target.

```bash
python3 /home/wt/rangen_ws/src/rangen_osc_bridge/scripts/osc_record.py --out /path/to/session_dir/session
```

Writes `session.jsonl` (full args, for `osc_to_midi.py`/`osc_replay.py`),
`session.csv` (Excel/pandas), and `session.txt` (Max text object + zl
slice) next to the given stem, creating the parent directory if needed.
The CSV/txt formats share the visualizer's fixed 3-arg-column schema, so
messages with more than 3 args (e.g. `/rangen/ee/quat`) are truncated
there — the JSONL always has the full list.

Useful flags:

```bash
python3 scripts/osc_record.py --out /path/to/session_dir/session --port 9998  # match a non-default port
```

Stop with Ctrl+C (SIGINT) or SIGTERM — files are flushed and closed on
shutdown. This is what
`/home/wt/demonstrations/scripts/record_demo.sh` launches and stops
automatically to capture OSC alongside each rosbag, writing into a
`<bag_name>_osc/` folder next to the bag.

## Convert a recording to MIDI (for Ableton)

After recording with `--record` above:

```bash
python3 /home/wt/rangen_ws/src/rangen_osc_bridge/scripts/osc_to_midi.py take01.jsonl
```

Uses `config/osc_to_midi.yaml` for the OSC-address → MIDI-CC mapping and
writes `take01.mid` next to the input file. Drag the `.mid` file onto an
Ableton MIDI track, use "Unfold" to see each CC lane, then MIDI Learn
(Cmd/Ctrl+M) to map a CC to a synth parameter.

Override output path, config, or thinning:

```bash
python3 scripts/osc_to_midi.py take01.jsonl -o session1.mid
python3 scripts/osc_to_midi.py take01.jsonl -c my_mapping.yaml
python3 scripts/osc_to_midi.py take01.jsonl --thin 20
```

## Known TODO

Pose source is hardcoded to `PoseStamped` on `/ee_pose_ref` (the commanded
teleop pose). Switching to the measured IGPS pose
(`/ground_truth/odometry_igps/gen3_robotiq_85_tool_link`, an `Odometry`
message) needs a code change in `ee_osc_bridge.py`, not just a config edit —
see the TODO comment near the top of that file.

The bridge only sends to one OSC target at a time, so the curve editor and
the visualizer/Ableton can't listen simultaneously (see "Tune normalization
curves" above). Sending to multiple targets at once (a list of ip:port
pairs in `ee_osc_bridge.yaml`) would remove the back-and-forth — not
implemented yet.

Only `vel_lin_mag` and `accel_lin_mag` go through a tunable curve. `pos`,
`quat`, and the per-axis `vel_lin` / `vel_ang` / `accel_lin` components are
still sent raw/un-normalized — extending curve support to those would mean
adding more channels to `_CHANNELS` in `curve_editor.py` and more curve
lookups in `ee_osc_bridge.py`'s `_send_osc`.

---

**Keep this README current.** Update it every time you change code, config,
or launch behaviour in this package — the goal is to be able to launch
everything here from this file alone, even after months away from it.
