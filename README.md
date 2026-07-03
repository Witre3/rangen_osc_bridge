# rangen_osc_bridge

OSC bridge: publishes Rangen end-effector pose, velocity, and acceleration
over OSC (UDP) for mapping into music software (Ableton Live, Max/MSP, etc).
See `/home/wt/rangen_ws/CLAUDE.md` (Composer Bridge section) for how this
fits into the wider Rangen × HoMeR project.

## What's in this package

| File | Role |
|---|---|
| `rangen_osc_bridge/ee_osc_bridge.py` | ROS2 node — subscribes to EE pose, derives velocity/acceleration, sends OSC |
| `rangen_osc_bridge/norm_curve.py` | Shared breakpoint-curve normalization (0-1), used by both the bridge and the curve editor |
| `scripts/osc_visualizer.py` | Standalone (no ROS2) live monitor — 3-D trail + vel/accel plots + OSC log |
| `scripts/curve_editor.py` | Standalone (no ROS2) GUI — drag/bend curve breakpoints for the `/mag/norm` outputs against live OSC data |
| `scripts/osc_to_midi.py` | Converts a recorded OSC session (JSONL) to a MIDI file for Ableton |
| `config/ee_osc_bridge.yaml` | OSC target IP/port, send rate, smoothing, pose topic, norm curve file path |
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
```

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
