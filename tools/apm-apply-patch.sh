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

n=0
for p in patches/*.patch; do
    [ -e "$p" ] || { echo "no patches in patches/"; exit 0; }
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
