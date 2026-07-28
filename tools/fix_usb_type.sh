#!/usr/bin/env bash
#
# fix_usb_type.sh — make an already-written exFAT stick readable by macOS.
#
# Symptom this fixes:
#   "The disk you inserted was not readable by this computer."
#
# Cause: the GPT partition *type GUID* is Linux filesystem data
# (0FC63DAF-8483-4772-8E79-3D69D8477DE4) rather than Microsoft basic data
# (EBD0A0A2-B9E5-4433-87C0-68B6B72699C7).  macOS refuses to probe a partition
# it does not recognise the type of, even when the exFAT inside is valid.
#
# NON-DESTRUCTIVE: this rewrites 16 bytes in the GPT partition entry.  The
# filesystem and everything already copied onto it are untouched.
#
#   sudo tools/fix_usb_type.sh /dev/sdX
#
set -euo pipefail

DEV="${1:-}"
WANT="ebd0a0a2-b9e5-4433-87c0-68b6b72699c7"

die() { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }
say() { printf '\033[1m==>\033[0m %s\n' "$*"; }

[ -n "$DEV" ] || die "usage: sudo $0 /dev/sdX   (the whole disk, not a partition)"
[ -b "$DEV" ] || die "$DEV is not a block device"
[ "$(id -u)" -eq 0 ] || die "must run as root (sudo)"

BASE="$(basename "$DEV")"
[ -e "/sys/block/$BASE" ] || die "$DEV is not a whole disk"
[ "$(cat "/sys/block/$BASE/removable")" = "1" ] || die "$DEV is not removable — refusing"
[ "$(lsblk -dno TRAN "$DEV")" = "usb" ] || die "$DEV is not USB — refusing"

PART="${DEV}1"; [ -b "$PART" ] || PART="${DEV}p1"
[ -b "$PART" ] || die "no first partition on $DEV"

say "before"
lsblk -o NAME,SIZE,FSTYPE,LABEL,PARTTYPE "$DEV"

BEFORE="$(lsblk -no PARTTYPE "$PART" | tr '[:upper:]' '[:lower:]' | tr -d ' ')"
if [ "$BEFORE" = "$WANT" ]; then
  say "already Microsoft basic data — nothing to change"
  say "so the unreadable-disk message has another cause; see the notes below"
  exit 0
fi

umount "$PART" 2>/dev/null || udisksctl unmount -b "$PART" 2>/dev/null || true

say "setting partition type to Microsoft basic data"
parted -s "$DEV" set 1 msftdata on
partprobe "$DEV"; sleep 2

AFTER="$(lsblk -no PARTTYPE "$PART" | tr '[:upper:]' '[:lower:]' | tr -d ' ')"
say "after"
lsblk -o NAME,SIZE,FSTYPE,LABEL,PARTTYPE "$DEV"

[ "$AFTER" = "$WANT" ] || die "type is still '$AFTER' — did not take"

say "done — the filesystem and its contents were not touched"
say "eject, replug into the Mac, and it should mount as RANGEN"
