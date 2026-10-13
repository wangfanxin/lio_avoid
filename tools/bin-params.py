#!/usr/bin/env python3
"""The parameters the FCU ACTUALLY ran with, read from its own log's PARM records.

Every configuration claim in the write-up is only as good as this: what we passed with `--defaults` and
what the firmware ended up with are two different things (MEASURED 2026-10-13: two `--defaults`
arguments do not accumulate -- the second REPLACES the first, and the FCU reported `EK3_SRC1_POSXY 3`
while we believed our file was loaded).  This reads the values out of the flight log instead.

usage: tools/bin-params.py <flight.BIN> [NAME ...]
"""
from __future__ import annotations

import sys

from pymavlink import mavutil

WATCH = ["EK3_SRC1_POSXY", "EK3_SRC1_POSZ", "EK3_SRC1_VELXY", "EK3_SRC1_VELZ", "EK3_SRC1_YAW",
         "VISO_TYPE", "GPS1_TYPE", "EK3_IMU_MASK", "INS_ENABLE_MASK", "LOG_DISARMED",
         "DISARM_DELAY", "ARMING_CHECK", "FRAME_CLASS", "FRAME_TYPE", "WP_SPD", "WP_ACC",
         "LOIT_SPEED_MS", "EK3_POSNE_M_NSE", "EK3_ALT_M_NSE", "SIM_RATE_HZ"]


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__.strip().splitlines()[-1])
        return 2
    path = sys.argv[1]
    want = sys.argv[2:] or WATCH
    mlog = mavutil.mavlink_connection(path)
    seen = {}
    while True:
        m = mlog.recv_match(type="PARM", blocking=False)
        if m is None:
            break
        name = m.Name if isinstance(m.Name, str) else m.Name.decode("ascii", "replace")
        seen[name] = m.Value
    if not seen:
        print("%s has no PARM records" % path)
        return 1
    print("parameters in %s (the firmware's own record, %d names)" % (path, len(seen)))
    missing = []
    for n in want:
        if n in seen:
            print("   %-18s %.4f" % (n, seen[n]))
        else:
            missing.append(n)
    if missing:
        print("   not in the log: %s" % " ".join(missing))
    return 0


if __name__ == "__main__":
    sys.exit(main())
