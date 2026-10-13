#!/usr/bin/env bash
# Mission A, N flights, with a summary that carries "how many of how many" for every acceptance line.
#
#   tools/run-mission-batch.sh [N] [TAG_PREFIX]
#
# Each flight is a full run-mission.sh; the per-flight metrics land in logs/missionA-<tag>/metrics.txt
# and the batch summary is written next to them.  A flight that aborts is COUNTED, with its reason --
# the whole point of the counts is that a number without its denominator is not a result.
set -u
N="${1:-10}"
PREFIX="${2:-batch}"
REPO=/mnt/d/distill_self/lio_avoid
BASE=/home/fanxin/lio_avoid/logs
STAMP="$(date +%Y%m%d_%H%M%S)"
SUMMARY="$BASE/missionA-$PREFIX-$STAMP-summary.txt"
mkdir -p "$BASE"

echo "batch: $N flights, prefix $PREFIX, summary -> $SUMMARY"
echo "batch $N flights $STAMP" > "$SUMMARY"

launched=0; completed=0; aborted=0; crashed=0
for i in $(seq 1 "$N"); do
    TAG="$PREFIX-$(printf '%02d' "$i")-$STAMP"
    echo
    echo "================ flight $i of $N ($TAG) ================"
    launched=$((launched+1))
    if timeout 300 bash "$REPO/tools/run-mission.sh" "$TAG" > /tmp/run-$TAG.log 2>&1; then
        rc=0
    else
        rc=$?
    fi
    D="$BASE/missionA-$TAG"
    phase_last=$(tail -1 "$D/phase.csv" 2>/dev/null | cut -d, -f3)
    fsm_abort=$(grep -c "ABORT" "$D/fsm.log" 2>/dev/null || echo 0)
    if [ "$rc" -eq 0 ] && [ "$phase_last" = "done" ]; then
        completed=$((completed+1))
        verdict="completed"
    elif [ "$rc" -eq 124 ]; then
        crashed=$((crashed+1))
        verdict="TIMEOUT"
    else
        aborted=$((aborted+1))
        verdict="aborted: $(grep -m1 'ABORT' "$D/fsm.log" 2>/dev/null | sed 's/.*ABORT: //')"
    fi
    echo "flight $i: $verdict (rc=$rc, last phase $phase_last)" | tee -a "$SUMMARY"
    grep -E "^A1|^A2|^A3|^B1|^\[PASS\]|^\[FAIL\]|corner [0-9]: closest|A1/A2/B1" \
        "$D/metrics.txt" 2>/dev/null | sed 's/^/    /' >> "$SUMMARY"
    cp "$D/metrics.txt" "$BASE/missionA-$PREFIX-$STAMP-flight$(printf '%02d' "$i").metrics.txt" \
        2>/dev/null || true
done

{
    echo
    echo "================ batch summary: $PREFIX $STAMP ================"
    echo "launched        $launched"
    echo "completed       $completed of $launched"
    echo "aborted         $aborted of $launched"
    echo "timed out       $crashed of $launched"
    echo
    echo "the acceptance lines, counted across the completed flights:"
    for pat in "^A1" "^A2" "^A3" "^B1"; do
        n=$(grep -h "$pat" "$BASE"/missionA-$PREFIX-$STAMP-flight*.metrics.txt 2>/dev/null | wc -l)
        echo "   $pat : present in $n of $completed completed flights"
    done
    echo
    echo "(every line above is per flight; read the per-flight blocks for the numbers)"
} | tee -a "$SUMMARY"

echo
echo "summary written to $SUMMARY"
