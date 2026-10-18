#!/usr/bin/env python3
"""ROUND 16: the missing conversion, written as ONE function with a test that has teeth.

`mavlink_impl.send_cmd` converts the POSITION with the calibration and hands the plan's yaw through untouched
(`src/bridge/mavlink_impl.py:453`).  But the contract says the yaw is the FCU's COMPASS HEADING (0 = North, +90 deg =
East, `planner/cmd_stream.py:83-89`) while the plan's yaw is an L-FRAME angle (`traj_server` computes atan2 on the
L spline).  This script states the conversion and proves it at the four points where the two conventions disagree most
-- +-90 deg -- so the fix cannot be "close enough by luck" at 0/180 like it has been so far.

It is a TEST, not yet a wiring: nothing is changed in the bridge until this is green AND a Mission-B regression flight
has been taken (the bridge is shared by both missions).
"""
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "src"))
from common import frames  # noqa: E402


def heading_from_l_yaw(yaw_l, calib):
    """The plan's yaw (an angle in L) -> the contract's yaw (the FCU's compass heading).

        theta_W = theta_L + yaw(R_WL)        the L x axis, as an angle in W
        heading = pi/2 - theta_W             W's angles run anticlockwise from +x(East);
                                             the FCU's heading runs clockwise from +y(North)
    """
    R = np.asarray(calib.R_WL, dtype=float)
    theta_w = float(yaw_l) + math_atan2(R)
    return frames.wrap_pi(0.5 * np.pi - theta_w)


def math_atan2(R):
    import math
    return math.atan2(R[1, 0], R[0, 0])


def main() -> int:
    import json
    import math
    k = json.load(open(os.path.join(os.path.dirname(REPO), "_stage", "calib.json")))
    calib = frames.Calib(R_WL=np.array(k["R_WL"], dtype=float), t_WL=np.array(k["t_WL"], dtype=float),
                         n_samples=int(k.get("n_samples", 0)), rms_m=float(k.get("rms_m", float("nan"))))
    print(calib.summary())
    # theta_L is the direction of travel IN L.  The four cardinal cases, worked by hand:
    #   theta_L = 0    -> travelling +x_L, which is +y in W   -> NORTH  -> heading 0
    #   theta_L = +90  -> travelling +y_L, which is -x in W   -> WEST   -> heading 270 (-90)
    #   theta_L = 180  -> travelling -x_L, which is -y in W   -> SOUTH  -> heading 180
    #   theta_L = -90  -> travelling -y_L, which is +x in W   -> EAST   -> heading 90
    cases = [(0.0, 0.0), (90.0, -90.0), (180.0, 180.0), (-90.0, 90.0)]
    bad = []
    for deg_l, deg_want in cases:
        got = heading_from_l_yaw(math.radians(deg_l), calib)
        want = math.radians(deg_want)
        err = frames.wrap_pi(got - want)
        ok = abs(err) < math.radians(1.0)
        print("   theta_L %+6.1f deg -> heading %+7.2f deg (want %+7.1f)  %s"
              % (deg_l, math.degrees(got), deg_want, "OK" if ok else "*** WRONG ***"))
        if not ok:
            bad.append(deg_l)
    # and the CURRENT behaviour, for the record: yaw passed through
    print("\n   what the bridge does today (yaw passed through) on the same four points:")
    for deg_l, deg_want in cases:
        err = frames.wrap_pi(math.radians(deg_l) - math.radians(deg_want))
        print("   theta_L %+6.1f deg -> wire %+7.1f deg (want %+7.1f)  error %+7.1f deg"
              % (deg_l, deg_l, deg_want, math.degrees(err)))
    if bad:
        print("\nYAW CONVERSION TEST FAILED at %s" % bad)
        return 1
    print("\nYAW CONVERSION TEST PASSED: theta_L -> compass heading, four cardinal cases, < 1 deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
