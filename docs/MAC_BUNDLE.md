# The macOS bundle — rebuilding it, and adding signals

The composer's Mac runs recorded takes with no ROS2, no Python and no install
step. `rangen_osc_bridge/mcap_osc_player.py` reads the `.mcap` directly, replays
it in wall-clock time and drives two outputs from one clock: OSC on UDP 9000,
and a Foxglove WebSocket the browser connects to. Because it is one clock, the
sound and the picture cannot drift — and since the player declares
`Capability.PlaybackControl`, the Foxglove seek bar drives *both*.

`docs/MAC_README.txt` is the collaborator-facing copy. This file is for us.

## Rebuild the bundle

```bash
tools/build_mac_bundle.sh          # -> dist/RangenPlayer/ and dist/RangenPlayer.zip
```

Runs entirely on Linux, compiles nothing, takes ~1 min warm. It downloads a
relocatable CPython from python-build-standalone for both Mac architectures and
cross-installs macOS wheels with `pip --platform`. Result is ~220 MB unpacked,
~108 MB zipped. Tarballs are cached in `~/.cache/rangen-mac-bundle`.

PyInstaller would be more idiomatic, but it can only build a macOS binary *on*
macOS and we have no Mac. This approach needs none.

Bumping Python or the pinned runtime: edit `PY_VER` / `PBS_TAG` at the top of
the script — check the tag exists at
<https://github.com/astral-sh/python-build-standalone/releases>.

## Ship it

```
USB stick (exFAT, label RANGEN)
  README.txt          copy of docs/MAC_README.txt
  RangenPlayer.zip    dragged to the Desktop, unzipped there
  bags/               the takes, left on the stick
```

exFAT is required: FAT32 caps files at 4 GB and the takes are ~4.8 GB. exFAT
also cannot store the executable bit, which is why the app ships as a **zip** —
unzipping onto the Desktop (APFS) restores `+x` on `Start Rangen.command`, and
unzipping on the stick would not. The launcher scans `./bags` *and*
`/Volumes/*/bags`, so the app on the Desktop still finds takes on the stick
without copying tens of GB.

Physical hand-off of a stick carries no `com.apple.quarantine`, so right-click →
Open is needed once at most. If the zip is instead emailed or sent via Drive,
quarantine applies: `xattr -dr com.apple.quarantine ~/Desktop/RangenPlayer`.

## Adding a new ROS topic to the OSC stream

**No code change, on either side.** Add a row to the `generic:` list in
`config/osc_signals.yaml`:

```yaml
generic:
  - topic:   /joint_states
    type:    sensor_msgs/msg/JointState
    field:   velocity[3]
    address: /rangen/joint_states/velocity3
    arity:   1
    enabled: true
```

The robot picks it up via `generic_sources.py` (hot-reloaded by mtime, no node
restart). The Mac picks it up via `mcap_sources.py`. Both call the same
`osc_signals.extract_floats()`, so the address carries the same numbers from
either source. Re-run `tools/build_mac_bundle.sh` to fold the new config into
the bundle.

The differences are only in how the message is obtained:

| | robot (`generic_sources.py`) | Mac (`mcap_sources.py`) |
|---|---|---|
| message source | DDS subscription | channel in the `.mcap` |
| type resolution | `rosidl` `get_message()` — needs the package sourced | schema embedded in the file — custom types just work |
| QoS | mirrors the publisher | n/a |
| missing topic | warn once, keep going | warn once, keep going |

That second row is why a `ranger_msgs` field works on a Mac that has never heard
of `ranger_msgs`.

## Structure

| File | Role |
|---|---|
| `ee_kinematics.py` | `EeState`, `OscSender`, `emit_builtin` — the 17 built-in addresses. No rclpy, **no numpy**. Shared by the node and the player, so the arithmetic cannot diverge. |
| `mcap_osc_player.py` | Replay, pacing, transport, Foxglove forwarding |
| `mcap_sources.py` | Generic-signal extraction from MCAP channels |
| `mac_launcher.py` | Bag picker + banner; what `Start Rangen.command` runs |
| `tools/build_mac_bundle.sh` | Assembles the bundle |

`ee_kinematics.py` is numpy-free deliberately: it keeps the bundle to wheels
that are pure Python or prebuilt for both Mac architectures, and the pure-Python
3-vector arithmetic is free at 50 Hz. It was verified bit-identical to the numpy
version it replaced over 20 000 samples.

Modules copied into the bundle are imported flat (`app/` is on `PYTHONPATH`, no
package), so each does `try: from .x import ... except ImportError: from x
import ...`. Adding a new shared module means adding that fallback *and* adding
it to `APP_MODULES` in the build script.

## Running it on Linux

Same script, no bundle:

```bash
pip install mcap mcap-ros2-support foxglove-sdk        # or: pip install .[mcap]
python3 -m rangen_osc_bridge.mcap_osc_player ~/homer/data/ENSEMBLE/<take>
```

Useful flags: `--speed`, `--loop`, `--osc-port`, `--ws-port`, `--no-foxglove`,
`--wait-for-viewer 0`, and `--topics /ee_pose_ref /tf /tf_static /joint_states`
to drop the camera streams if playback stutters (an uncompressed
`/gen3/rgb/image_rect` take pushes ~27 MB/s through the socket).
