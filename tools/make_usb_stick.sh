#!/usr/bin/env bash
#
# make_usb_stick.sh — one command to produce the collaborator's USB stick.
#
# Wraps the two steps that actually do the work:
#   tools/build_mac_bundle.sh   builds dist/RangenPlayer.zip (as you, not root)
#   tools/prepare_usb.sh        formats exFAT and copies (root, asks for ERASE)
#
# Run it as yourself; it only escalates for the formatting step, so the build
# cache and dist/ stay owned by you rather than by root.
#
#   bash tools/make_usb_stick.sh
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TAKES_DIR="${RANGEN_TAKES_DIR:-$HOME/homer/data/ENSEMBLE}"
ZIP="$REPO/dist/RangenPlayer.zip"

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

[ "$(id -u)" -ne 0 ] || die "run this as yourself, not with sudo — it will ask when it needs to"

# ── 1. the bundle ────────────────────────────────────────────────────────────
NEWEST_SRC="$(find "$REPO/rangen_osc_bridge" "$REPO/config" "$REPO/docs" -type f \
              -newer "$ZIP" -print -quit 2>/dev/null || true)"
if [ ! -f "$ZIP" ] || [ -n "$NEWEST_SRC" ]; then
  bold "Building the Mac bundle (a minute or so)..."
  "$REPO/tools/build_mac_bundle.sh" >/dev/null
else
  bold "Bundle is up to date: $(du -h "$ZIP" | cut -f1)"
fi

# ── 2. the stick ─────────────────────────────────────────────────────────────
mapfile -t DISKS < <(
  for d in /sys/block/*; do
    n="$(basename "$d")"
    [ -e "$d/removable" ] || continue
    [ "$(cat "$d/removable")" = "1" ] || continue
    [ "$(lsblk -dno TRAN "/dev/$n" 2>/dev/null)" = "usb" ] || continue
    printf '%s\t%s\t%s\n' "/dev/$n" \
      "$(lsblk -dno SIZE "/dev/$n")" \
      "$(lsblk -no LABEL "/dev/$n" | grep -v '^$' | head -1 | tr -d '\n')"
  done
)

[ "${#DISKS[@]}" -gt 0 ] || die "no removable USB disk found — plug the stick in"

echo
bold "USB sticks found:"
for i in "${!DISKS[@]}"; do
  IFS=$'\t' read -r dev size label <<<"${DISKS[$i]}"
  printf '  %d)  %-10s %-8s %s\n' "$((i+1))" "$dev" "$size" "${label:-(no label)}"
done
echo

if [ "${#DISKS[@]}" -eq 1 ]; then
  IFS=$'\t' read -r DEV DEV_SIZE _ <<<"${DISKS[0]}"
  printf 'Using the only stick found: %s (%s)\n' "$DEV" "$DEV_SIZE"
else
  read -r -p "Which stick? [1-${#DISKS[@]}] " pick
  [[ "$pick" =~ ^[0-9]+$ ]] && [ "$pick" -ge 1 ] && [ "$pick" -le "${#DISKS[@]}" ] \
    || die "not a valid choice"
  IFS=$'\t' read -r DEV DEV_SIZE _ <<<"${DISKS[$((pick-1))]}"
fi

# GiB, to match `du -BG` below -- mixing decimal GB with GiB would make the
# capacity check optimistic by ~7%, which is a whole take on a full stick.
DEV_GB="$(( $(lsblk -bdno SIZE "$DEV") / 1073741824 ))"

# ── 3. the takes ─────────────────────────────────────────────────────────────
mapfile -t TAKE_PATHS < <(
  find "$TAKES_DIR" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | sort
)
[ "${#TAKE_PATHS[@]}" -gt 0 ] || die "no takes found in $TAKES_DIR (override with RANGEN_TAKES_DIR)"

echo
bold "Takes available in $TAKES_DIR:"
declare -a TAKE_GB
for i in "${!TAKE_PATHS[@]}"; do
  gb="$(du -sBG "${TAKE_PATHS[$i]}" 2>/dev/null | cut -f1 | tr -d 'G')"
  TAKE_GB[$i]="$gb"
  printf '  %d)  %-6s %s\n' "$((i+1))" "${gb}G" "$(basename "${TAKE_PATHS[$i]}")"
done
echo
printf 'Stick holds about %d GB. Pick the takes to copy.\n' "$DEV_GB"
read -r -p "Numbers separated by spaces (or 'all', or Return for none): " -a PICKS

SELECTED=()
TOTAL=0
if [ "${PICKS[*]:-}" = "all" ]; then
  SELECTED=("${TAKE_PATHS[@]}")
  for g in "${TAKE_GB[@]}"; do TOTAL=$((TOTAL + g)); done
elif [ "${#PICKS[@]}" -gt 0 ]; then
  for p in "${PICKS[@]}"; do
    [ -n "$p" ] || continue
    [[ "$p" =~ ^[0-9]+$ ]] && [ "$p" -ge 1 ] && [ "$p" -le "${#TAKE_PATHS[@]}" ] \
      || die "'$p' is not one of the numbers above"
    SELECTED+=("${TAKE_PATHS[$((p-1))]}")
    TOTAL=$((TOTAL + TAKE_GB[$((p-1))]))
  done
fi

if [ "$TOTAL" -gt "$((DEV_GB - 1))" ]; then
  die "those takes total ~${TOTAL} GB but the stick holds ~${DEV_GB} GB — pick fewer"
fi

# ── 4. hand over ─────────────────────────────────────────────────────────────
echo
bold "About to write:"
printf '  device : %s (%s) — WILL BE ERASED\n' "$DEV" "$DEV_SIZE"
printf '  player : %s\n' "$(du -h "$ZIP" | cut -f1)"
if [ "${#SELECTED[@]}" -eq 0 ]; then
  printf '  takes  : none (the collaborator can drop .mcap files into bags/ later)\n'
else
  printf '  takes  : %d, about %d GB\n' "${#SELECTED[@]}" "$TOTAL"
  for s in "${SELECTED[@]}"; do printf '           %s\n' "$(basename "$s")"; done
fi
echo
bold "Formatting needs root; sudo will ask below, then type ERASE to confirm."
echo

exec sudo "$REPO/tools/prepare_usb.sh" "$DEV" "${SELECTED[@]}"
