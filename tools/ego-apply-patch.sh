#!/usr/bin/env bash
# Apply (or revert) every patch in patches/ against the ego-planner tree.
#
# Same convention as tools/apm-apply-patch.sh: the third-party checkout stays PRISTINE in git, and
# the whole of our change to it is one file to read and one command to reproduce.  `--check` runs
# first, so a patch that does not apply cleanly stops the loop instead of leaving a half-patched tree.
#
# usage: tools/ego-apply-patch.sh [--revert] [EGO_TREE]
#        EGO_TREE defaults to /home/fanxin/lio_avoid/src/ego-planner-swarm
set -eu

EGO_TREE=/home/fanxin/lio_avoid/src/ego-planner-swarm
REVERT=""
if [ "${1:-}" = "--revert" ]; then
    REVERT="--reverse"
    shift
fi
if [ "${1:-}" != "" ]; then
    EGO_TREE="$1"
fi

HERE="$(cd "$(dirname "$0")/.." && pwd)"
cd "$HERE"

if [ ! -d "$EGO_TREE/.git" ]; then
    echo "no ego-planner checkout at $EGO_TREE" >&2
    exit 1
fi

echo "ego-planner: $EGO_TREE"
echo "revision   : $(git -C "$EGO_TREE" rev-parse --short HEAD) ($(git -C "$EGO_TREE" rev-parse --abbrev-ref HEAD))"
echo

# THE ORDER IS PART OF THE OPERATION.  The series is a STACK: patch 0002 rewrites lines that 0009 also
# rewrites, so applying `--reverse` to 0002 FIRST asks it to undo context that the later patches have already
# replaced.  MEASURED 2026-10-15 (the acceptance test of this repair): forward application reproduced the live
# tree byte-for-byte on all three files, and `--revert` then died at the second patch --
#     == patches/0002-...  error: patch failed: ego_replan_fsm.cpp:243  ... dirty after revert: 3
# because the loop walked the list forwards.  Forward: in order.  Revert: in REVERSE order.
PATCHES=()
for p in patches/00[0-9][0-9]-*.patch; do
    [ -e "$p" ] || { echo "no 00NN-*.patch patches in patches/"; exit 0; }
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
    git -C "$EGO_TREE" apply --check $REVERT "$HERE/$p"
    git -C "$EGO_TREE" apply $REVERT "$HERE/$p"
    echo "   applied"
    n=$((n + 1))
done

echo
echo "$n patch(es) ${REVERT:+reverted}${REVERT:-applied}"
echo
echo "== the tree's diff against the pinned commit (this is the whole of our change) =="
git -C "$EGO_TREE" --no-pager diff --stat
