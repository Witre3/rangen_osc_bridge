#!/usr/bin/env bash
#
# build_mac_bundle.sh — assemble the macOS "Rangen Player" bundle, on Linux.
#
# The collaborator's Mac gets no ROS2, no Homebrew, no pip and no Python: the
# bundle carries its own interpreter and its own dependencies.  PyInstaller
# would be the idiomatic way to do that, but PyInstaller can only build a macOS
# binary on a macOS host and there isn't one -- so instead this drops in a
# relocatable CPython from python-build-standalone and pip-installs macOS wheels
# into it with pip's --platform cross-install.  Everything here runs on Linux
# and nothing is compiled.
#
# Both architectures ship; the launcher picks with `uname -m`.  Apple Silicon is
# arm64, pre-2020 Intel Macs are x86_64, and a wrong guess is a bundle that
# won't start at all.
#
# Usage:  tools/build_mac_bundle.sh [output_dir]
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$REPO/dist}"
CACHE="${RANGEN_BUILD_CACHE:-$HOME/.cache/rangen-mac-bundle}"

PY_VER="3.12.13"
PBS_TAG="20260718"
PBS_BASE="https://github.com/astral-sh/python-build-standalone/releases/download/${PBS_TAG}"

# Modules the player needs.  Deliberately only the ROS-free ones: importing
# ee_osc_bridge.py would pull in rclpy, which does not exist on that machine.
APP_MODULES=(
  ee_kinematics.py
  mcap_sources.py
  mcap_osc_player.py
  mac_launcher.py
  norm_curve.py
  osc_signals.py
)
CONFIGS=(osc_signals.yaml norm_curves.yaml ee_osc_bridge.yaml)
PIP_PKGS=(mcap mcap-ros2-support foxglove-sdk python-osc pyyaml)

STAGE="$OUT/RangenPlayer"

say() { printf '\033[1m==>\033[0m %s\n' "$*"; }

command -v curl >/dev/null || { echo "curl is required"; exit 1; }
command -v zip  >/dev/null || { echo "zip is required"; exit 1; }
python3 -c 'import pip' 2>/dev/null || { echo "python3 -m pip is required"; exit 1; }

say "output: $STAGE"
rm -rf "$STAGE"
mkdir -p "$STAGE" "$CACHE"

for PAIR in "arm64:aarch64:macosx_11_0_arm64" "x86_64:x86_64:macosx_10_13_x86_64"; do
  ARCH="${PAIR%%:*}"; REST="${PAIR#*:}"
  PBS_ARCH="${REST%%:*}"; PLAT="${REST#*:}"

  TARBALL="cpython-${PY_VER}+${PBS_TAG}-${PBS_ARCH}-apple-darwin-install_only_stripped.tar.gz"
  if [ ! -f "$CACHE/$TARBALL" ]; then
    say "downloading $TARBALL"
    curl -fL --retry 3 -o "$CACHE/$TARBALL.part" "$PBS_BASE/$TARBALL"
    mv "$CACHE/$TARBALL.part" "$CACHE/$TARBALL"
  else
    say "using cached $TARBALL"
  fi

  say "unpacking runtime -> python-$ARCH"
  rm -rf "$STAGE/.tmp"; mkdir -p "$STAGE/.tmp"
  tar -xzf "$CACHE/$TARBALL" -C "$STAGE/.tmp"
  mv "$STAGE/.tmp/python" "$STAGE/python-$ARCH"
  rm -rf "$STAGE/.tmp"
  # Trim what a player never uses; keeps the two-arch bundle manageable.
  rm -rf "$STAGE/python-$ARCH/lib/python3.12/test" \
         "$STAGE/python-$ARCH/lib/python3.12/idlelib" \
         "$STAGE/python-$ARCH/lib/python3.12/tkinter" \
         "$STAGE/python-$ARCH/lib/python3.12/turtledemo" \
         "$STAGE/python-$ARCH/share" 2>/dev/null || true

  say "installing macOS wheels ($PLAT) -> site-$ARCH"
  python3 -m pip install --quiet --disable-pip-version-check \
      --target "$STAGE/site-$ARCH" \
      --platform "$PLAT" --python-version "${PY_VER%.*}" \
      --only-binary=:all: --no-compile \
      "${PIP_PKGS[@]}"
  find "$STAGE/site-$ARCH" -name '*.dist-info' -prune -exec rm -rf {} + 2>/dev/null || true
done

say "copying player"
mkdir -p "$STAGE/app" "$STAGE/config" "$STAGE/bags"
for m in "${APP_MODULES[@]}"; do
  cp "$REPO/rangen_osc_bridge/$m" "$STAGE/app/$m"
done
for c in "${CONFIGS[@]}"; do
  cp "$REPO/config/$c" "$STAGE/config/$c"
done
cat > "$STAGE/bags/PUT_RECORDINGS_HERE.txt" <<'EOF'
Put the robot recordings in this folder.

Each recording is either a single .mcap file, or a folder that contains one.
Copy them in, then start the player again and they will show up in the list.
EOF

say "writing launcher"
cat > "$STAGE/Start Rangen.command" <<'EOF'
#!/bin/bash
# Double-click this file.  It opens Terminal, plays a robot recording, shows it
# in Foxglove in Chrome, and sends the OSC stream to port 9000.
cd "$(dirname "$0")" || exit 1

ARCH="$(uname -m)"
[ "$ARCH" = "arm64" ] || ARCH="x86_64"
PY="./python-$ARCH/bin/python3"

if [ ! -x "$PY" ]; then
  echo "This Mac reports '$(uname -m)', which this copy does not include."
  echo "Ask for a bundle built for that machine."
  read -r -p "Press Return to close. "
  exit 1
fi

export PYTHONPATH="$PWD/site-$ARCH:$PWD/app"
export RANGEN_HOLD_WINDOW=1
exec "$PY" -s "$PWD/app/mac_launcher.py" "$@"
EOF
chmod +x "$STAGE/Start Rangen.command"

cp "$REPO/docs/MAC_README.txt" "$STAGE/README.txt" 2>/dev/null || true

say "zipping"
( cd "$OUT" && rm -f RangenPlayer.zip && zip -qr RangenPlayer.zip RangenPlayer )

say "done"
du -sh "$STAGE" "$OUT/RangenPlayer.zip"
