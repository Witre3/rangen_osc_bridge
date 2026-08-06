# first_time_setup.md — rangen_osc_bridge on a new machine

Host-level setup that does not arrive with a clone. See `CLAUDE.md` for
orientation and `README.md` for the full reference.

Anything marked **HUMAN** needs an account, hardware, or a network decision.

---

## 1. Clone

```bash
git clone git@github.com:Witre3/rangen_osc_bridge.git ~/rangen_ws/src/rangen_osc_bridge
```

**This repo is on GitHub**, not `git.initrobots.ca` like the rest of the
workspace, and its remote is named `github` rather than `origin`.

**HUMAN:** an SSH key registered on GitHub.

## 2. Python dependencies

The standalone tools under `scripts/` must work on a machine with **no ROS**
(that is their reason for existing), so they are plain pip installs:

```bash
pip install --break-system-packages python-osc matplotlib numpy pyyaml
```

Per-tool extras, install only what you need:

```bash
pip install --break-system-packages mido python-rtmidi   # osc_to_midi.py
pip install --break-system-packages mcap mcap-ros2-support foxglove-sdk  # mcap_osc_player.py
```

For the ROS nodes (`ee_osc_bridge`, `state_interpreter`) build in the
workspace instead:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/rangen_ws
rosdep install --from-paths src/rangen_osc_bridge --ignore-src -r -y
colcon build --symlink-install --packages-select rangen_osc_bridge
source install/setup.bash
```

`state_interpreter` needs `tf2_ros` (declared in `package.xml`, so rosdep
handles it) and a **live TF tree** — it publishes nothing until TF resolves,
by design. A missing transform sends silence, not a zero.

## 3. Network — the part that is not automatable

OSC is UDP to a target IP and port set in `config/ee_osc_bridge.yaml`
(`osc_target_ip` / `osc_target_port`, plus a secondary `_2` pair that
`osc_record.py` listens on, default `127.0.0.1:9004`).

**HUMAN decisions:**

- The IP of the machine running Ableton/Max. On a different subnet or over
  VPN, UDP will be silently dropped — there is no error, the composer just
  hears nothing. Test with `scripts/osc_visualizer.py` on the target machine
  before blaming the bridge.
- Firewall openings for those UDP ports.

Override without editing the file:

```bash
ros2 launch rangen_osc_bridge ee_osc_bridge.launch.py \
  osc_target_ip:=192.168.1.42 osc_target_port:=9001
```

## 4. Verify

```bash
# no ROS needed — run this on the receiving machine
python3 scripts/osc_visualizer.py

# with ROS, both nodes:
ros2 launch rangen_osc_bridge ee_osc_bridge.launch.py
ros2 topic list | grep /rangen/state/     # 3 topics: elbow_state, reach, reach_discrete

# no TF tree available (e.g. replaying a bag):
ros2 launch rangen_osc_bridge ee_osc_bridge.launch.py state_interpreter:=false
```

The visualizer should show a moving 3-D trail plus the reach panel with its
near/far threshold lines. Nothing arriving means the network, not the bridge —
see §3.
