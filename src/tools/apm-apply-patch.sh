#!/usr/bin/env bash
# Apply (or revert) every patch in patches/ against the ArduPilot tree.
#
# The ArduPilot checkout stays PRISTINE in git: our only change to it is the truth tap, kept as a
# patch in this repo, so "what did we change in the flight code" is one file to read and one command
# to reproduce.  `--check` runs first, so a patch that does not apply cleanly stops the loop instead
# of leaving a half-patched tree.
#
# usage: tools/apm-apply-patch.sh [--revert] [AP_TREE]
#        AP_TREE defaults to /home/fanxin/lio_avoid/src/ardupilot
set -eu

AP_TREE=/home/fanxin/lio_avoid/src/ardupilot
REVERT=""
if [ "${1:-}" = "--revert" ]; then
    REVERT="--reverse"
    shift
fi
if [ "${1:-}" != "" ]; then
    AP_TREE="$1"
fi

HERE="$(cd "$(dirname "$0")/.." && pwd)"
cd "$HERE"

if [ ! -d "$AP_TREE/.git" ]; then
    echo "no ArduPilot checkout at $AP_TREE" >&2
    exit 1
fi

echo "ArduPilot: $AP_TREE"
echo "revision : $(git -C "$AP_TREE" describe --tags --always) $(git -C "$AP_TREE" rev-parse --short HEAD)"
echo

# THE GLOB IS NOT `patches/*.patch`, AND THAT IS THE WHOLE POINT.  patches/ once held patches for TWO
# trees: the ArduPilot truth tap (`apm-*.patch`) and the ego-planner series (`00NN-*.patch`, DELETED
# 2026-10-15 with the rest of the dropped line).  MEASURED 2026-10-15: with `patches/*.patch` this script
# tried to apply all eight ego patches to the ArduPilot tree (and the ego patcher's `00[0-9][0-9]-*` tried
# to apply the ArduPilot patch to the ego tree, where it died on
# `libraries/SITL/SIM_Aircraft.cpp: No such file or directory` and -- because of `set -e` -- left the ego
# series with NOTHING applied).  Each script selected its own tree's patches by name, and this one still
# does: `apm-*.patch` is now the only family left, and a future patch series gets its own prefix.
# The reverse order is the same rule the ego script used: a stack comes off the top.
PATCHES=()
for p in patches/apm-*.patch; do
    [ -e "$p" ] || { echo "no apm-*.patch in patches/"; exit 0; }
    PATCHES+=("$p")
done
ORDER=("${PATCHES[@]}")
if [ -n "$REVERT" ]; then
    ORDER=()
    for ((i = ${#PATCHES[@]} - 1; i >= 0; i--)); do ORDER+=("${PATCHES[$i]}"); done
fi

n=0
for p in "${ORDER[@]}"; do
    echo "== $p"
    git -C "$AP_TREE" apply --check $REVERT "$HERE/$p"
    git -C "$AP_TREE" apply $REVERT "$HERE/$p"
    echo "   applied"
    n=$((n + 1))
done

echo
echo "$n patch(es) ${REVERT:+reverted}${REVERT:-applied}"
echo
echo "== the tree's diff against the pinned commit (this is the whole of our change) =="
git -C "$AP_TREE" --no-pager diff --stat
