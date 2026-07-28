#!/usr/bin/env bash
#
# prepare_usb.sh — format a USB stick as exFAT and lay the Mac bundle onto it.
#
# exFAT is not a preference: FAT32 caps files at 4 GB and the takes are ~4.8 GB,
# and macOS reads/writes exFAT natively with no driver.  The player ships as a
# ZIP because exFAT cannot store the executable bit -- the collaborator drags
# the zip to the Desktop (APFS) and unzips there, which restores +x on
# "Start Rangen.command".  Unzipping on the stick would leave it un-clickable.
#
# DESTRUCTIVE.  Everything on the target device is erased.
#
# Usage:  sudo tools/prepare_usb.sh /dev/sdX [take_dir ...]
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEV="${1:-}"
shift || true
TAKES=("$@")

LABEL="RANGEN"
ZIP="$REPO/dist/RangenPlayer.zip"
README="$REPO/docs/MAC_README.txt"

die() { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }
say() { printf '\033[1m==>\033[0m %s\n' "$*"; }

[ -n "$DEV" ] || die "usage: sudo $0 /dev/sdX [take_dir ...]"
[ -b "$DEV" ] || die "$DEV is not a block device"
[ "$(id -u)" -eq 0 ] || die "must run as root (sudo)"
[ -f "$ZIP" ] || die "$ZIP missing — run tools/build_mac_bundle.sh first"

BASE="$(basename "$DEV")"
# Refuse anything that isn't a removable USB whole-disk.  A typo here would
# otherwise wipe an internal drive.
[ -e "/sys/block/$BASE" ] || die "$DEV is not a whole disk (don't pass a partition)"
[ "$(cat "/sys/block/$BASE/removable")" = "1" ] || die "$DEV is not removable — refusing"
[ "$(lsblk -dno TRAN "$DEV")" = "usb" ] || die "$DEV is not USB — refusing"

say "target device"
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINT "$DEV"
echo
printf '\033[31mThis ERASES everything above, permanently.\033[0m\n'
read -r -p "Type ERASE to continue: " CONFIRM
[ "$CONFIRM" = "ERASE" ] || die "aborted"

say "unmounting"
for part in $(lsblk -lno NAME "$DEV" | tail -n +2); do
  umount "/dev/$part" 2>/dev/null || true
done

say "partitioning + formatting exFAT"
wipefs -a "$DEV" >/dev/null
parted -s "$DEV" mklabel gpt
parted -s -a optimal "$DEV" mkpart "$LABEL" 1MiB 100%
# macOS will not open the partition unless its GPT *type GUID* is Microsoft
# basic data (EBD0A0A2-B9E5-4433-87C0-68B6B72699C7).  `parted mkpart` with no
# fs-type argument leaves it as Linux filesystem data (0FC63DAF-...), and the
# Mac then reports "The disk you inserted was not readable by this computer"
# even though the exFAT inside is perfectly valid.  This flag sets that GUID.
parted -s "$DEV" set 1 msftdata on
partprobe "$DEV"; sleep 2
PART="${DEV}1"; [ -b "$PART" ] || PART="${DEV}p1"
[ -b "$PART" ] || die "partition did not appear"
mkfs.exfat -n "$LABEL" "$PART" >/dev/null

# Fail loudly here rather than in front of the collaborator.
PTYPE="$(lsblk -no PARTTYPE "$PART" 2>/dev/null || true)"
case "$PTYPE" in
  ebd0a0a2-b9e5-4433-87c0-68b6b72699c7|EBD0A0A2-B9E5-4433-87C0-68B6B72699C7) ;;
  *) die "partition type is '$PTYPE', not Microsoft basic data — macOS will not read this" ;;
esac

say "mounting"
MNT="$(mktemp -d)"
mount "$PART" "$MNT"
trap 'umount "$MNT" 2>/dev/null || true; rmdir "$MNT" 2>/dev/null || true' EXIT

say "copying player"
cp "$ZIP" "$MNT/RangenPlayer.zip"
cp "$README" "$MNT/README.txt"
mkdir -p "$MNT/bags"

for take in "${TAKES[@]}"; do
  [ -e "$take" ] || { echo "  skipping missing $take"; continue; }
  say "copying take $(basename "$take")  ($(du -sh "$take" | cut -f1))"
  cp -r "$take" "$MNT/bags/"
done

sync
say "done — contents of $LABEL:"
ls -la "$MNT"
df -h "$MNT" | tail -1
