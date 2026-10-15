#!/usr/bin/env python3
"""A UNIFORM B-SPLINE EVALUATOR -- the arithmetic that turns ego-planner's output into positions.

WHERE IT COMES FROM.  ego publishes its trajectory as `traj_utils/msg/Bspline`: `order` (always 3), `knots`
(seconds, and they start NEGATIVE: `knots[0] = -order * interval`, `uniform_bspline.cpp:25-40`), `pos_pts`
(CONTROL points, not interpolated waypoints, in metres), and `start_time` (a SYSTEM-clock stamp).  The five
functions below restore code first written and verified in the Mission B round (`--selftest` at
`_stage/snapshots/0712-cleanup1/ego_adapter.py:319`); the in-tree original was deleted with the ego line
(`1ac78f8`).

HOW FAR THE VERIFICATION GOES, because "verified" is a word that has to mean something.  Three independent
references, on TWO cases (a hand-known straight line, and a knot vector whose interior spans differ):
a Cox-de Boor basis recursion (a different algorithm, not a restatement), a central difference of the
position against the analytic derivative spline, and the domain end against its textbook definition.  And
the whole thing is ATTACKED rather than restated: `tools/test-bspline-mutations.py` injects nine single-token
defects -- a missing clamp, a wrong domain end, an inclusive span search, one de Boor level too few, a
mis-indexed alpha, a mis-indexed derivative knot difference, a dropped degree factor, an unreduced knot
vector, a wrong `getTimeSum` -- and requires every one of them to make the selftest FAIL.  The first version
of this selftest let THREE of the nine survive; the second case and the difference check exist because of
that measurement, not because they looked tidy.

WHAT IT DOES NOT DO: no frame, no clock, no policy.  `t` is TRAJECTORY-relative seconds (0 at the start of
the domain) and the points are in whatever frame the planner was fed.  The caller owns both -- in this stack
that is `planner/ego_source.py` (L, and `t = now - start_time`, clamped, which is also what ego's own
`traj_server.cpp:170-216` does).

`bspline_derivs` returns (position, velocity, acceleration) by evaluating the derivative splines, which is
the same construction ego uses (`traj_[1] = traj_[0].getDerivative()` chains, `traj_server.cpp:63-64`) --
there is no `getVelocity(t)` on ego's class either.
"""
from __future__ import annotations

import numpy as np  # noqa: F401  (the lifted functions use it)

def bspline_eval(knots, pts, order, t):
    """`ego_planner::UniformBspline::evaluateDeBoorT(t)`, transliterated.

    The planner sends (order, knots, pos_pts) in `traj_utils/msg/Bspline`; `traj_server.cpp` rebuilds
    the spline with `UniformBspline pos_traj(pos_pts, order, 0.1); pos_traj.setKnot(knots);` and calls
    `evaluateDeBoorT(t)`, which is `evaluateDeBoor(t + u_(p_))`.  This is that function, and the
    self-test checks it against the Cox-de Boor basis recursion rather than against a re-reading of
    the same C++ -- a port that agrees with itself proves nothing.
    """
    u = np.asarray(knots, dtype=float)
    P = np.asarray(pts, dtype=float)
    p = int(order)
    n = P.shape[0] - 1
    m = n + p + 1
    if u.shape[0] != m + 1:
        raise ValueError("knots %d != n+p+2 = %d for %d control points and order %d"
                         % (u.shape[0], m + 1, P.shape[0], p))
    ub = min(max(u[p], t + u[p]), u[m - p])

    k = p
    while u[k + 1] < ub:                 # C++: `if (u_(k+1) >= ub) break;`
        k += 1
        if k >= m:
            break

    d = [P[k - p + i].copy() for i in range(p + 1)]
    for r in range(1, p + 1):
        for i in range(p, r - 1, -1):
            den = u[i + 1 + k - r] - u[i + k - p]
            alpha = 0.0 if den == 0.0 else (ub - u[i + k - p]) / den
            d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i]
    return d[p]


def bspline_duration(knots, n_ctrl, order):
    """`getTimeSum()` = u_(m_-p_) - u_(p_) with m_ = (n_ctrl-1) + order + 1."""
    u = np.asarray(knots, dtype=float)
    m = (n_ctrl - 1) + int(order) + 1
    return float(u[m - int(order)] - u[int(order)])


def _bspline_deriv_ctrl(knots, pts, order):
    """(knots', ctrl') of the EXACT first derivative spline of a B-spline.

    Standard result (The NURBS Book 3.3, eq. 3.8): the derivative of a degree-p B-spline is a
    degree-(p-1) B-spline on the reduced knot vector u[1:-1] with control points
        Q_i = p * (P_(i+1) - P_i) / (u_(i+p+1) - u_(i+1)),   i = 0 .. n-1.
    `UniformBspline::getDerivative()` in ego-planner is exactly this, so the numbers below are the
    planner's own definition of velocity and acceleration -- not a re-derivation of something else.
    A zero denominator (a repeated knot) contributes a zero control point, which is the same
    convention de Boor's algorithm uses for the same case.
    """
    u = np.asarray(knots, dtype=float)
    P = np.asarray(pts, dtype=float)
    p = int(order)
    n = P.shape[0] - 1                       # the derivative has n control points
    num = P[1:] - P[:-1]
    den = u[p + 1:p + 1 + n] - u[1:1 + n]
    Q = np.zeros_like(num)
    ok = np.abs(den) > 1e-12
    Q[ok] = (float(p) / den[ok])[:, None] * num[ok]
    return u[1:-1], Q


def bspline_derivs(knots, pts, order, t):
    """(position, velocity, acceleration) of the uniform B-spline at trajectory time t, EXACTLY.

    WHY THIS EXISTS.  The Path interface is positions only, so `fsm/mission_fsm.py` had to recover the
    velocity by differencing consecutive poses on the ~20 ms sample grid and the acceleration by a
    second difference (`ref_at`).  Those are SEGMENT averages, not the value at t: at 50 Hz and 1 m/s the
    discretisation is small, and at 3 m/s -- the owner's demand -- the reference the FCU is streamed has
    the wrong SHAPE exactly where the plan turns hardest (docs/ARCHITECTURE.md, "exact enough at 1 m/s
    and the wrong shape at 3 m/s").  The control points and the knot vector are already in the Bspline
    message, so the exact derivative costs nothing and is what this function returns.

    Timing convention: `t` is the same relative trajectory time `bspline_eval` takes (0 at the start of
    the domain u_(p_), `bspline_duration()` at its end), because the derivative spline is evaluated on
    the reduced knot vector, whose own domain starts at u_(p_) as well.
    """
    p = int(order)
    pos = bspline_eval(knots, pts, p, t)
    vel = np.zeros(3)
    acc = np.zeros(3)
    if p >= 1:
        u1, Q = _bspline_deriv_ctrl(knots, pts, p)
        vel = bspline_eval(u1, Q, p - 1, t)
        if p >= 2:
            u2, R = _bspline_deriv_ctrl(u1, Q, p - 1)
            acc = bspline_eval(u2, R, p - 2, t)
    return pos, vel, acc


def cox_de_boor(knots, pts, order, t):
    """Independent reference: find-span + BasisFuns (The NURBS Book A2.1/A2.2), then sum.

    A different ALGORITHM from de Boor's corner-cutting (it never evaluates a point combination
    recursively), which is what makes it a check rather than a restatement.  It also has to be
    written correctly at the domain end: a first version of this reference special-cased `x == hi`
    with `u[i] < x <= u[i+1]`, which made TWO spans contribute at the last sample and reported a
    2.25 m disagreement with a de Boor evaluator that was in fact right.  The span search below is
    the textbook one and has no such case.
    """
    u = np.asarray(knots, dtype=float)
    P = np.asarray(pts, dtype=float)
    p = int(order)
    n = P.shape[0] - 1
    m = n + p + 1
    lo, hi = u[p], u[m - p]
    x = min(max(t + lo, lo), hi)

    if x >= u[n + 1]:                        # the last non-empty span at the domain end
        k = n
    else:
        a, b = p, n + 1
        while b - a > 1:
            mid = (a + b) // 2
            if x < u[mid]:
                b = mid
            else:
                a = mid
        k = a

    N = np.zeros(p + 1)
    N[0] = 1.0
    left = np.zeros(p + 1)
    right = np.zeros(p + 1)
    for j in range(1, p + 1):
        left[j] = x - u[k + 1 - j]
        right[j] = u[k + j] - x
        s = 0.0
        for r in range(j):
            den = right[r + 1] + left[j - r]
            tmp = 0.0 if den == 0.0 else N[r] / den
            N[r] = s + right[r + 1] * tmp
            s = left[j - r] * tmp
        N[j] = s

    out = np.zeros(P.shape[1])
    for j in range(p + 1):
        out += N[j] * P[k - p + j]
    return out


def _linear_case():
    """Cubic B-spline through collinear, evenly spaced control points: a straight line, constant speed.

    The knots follow ego's convention (`knots[0] = -order * interval`), so this also pins the domain
    arithmetic that a wrong `knots[0]` would silently shift.
    """
    order, interval = 3, 0.5
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    knots = np.array([(i - order) * interval for i in range(len(pts) + order + 1)])
    return order, interval, pts, knots


def _nonuniform_case():
    """Six control points, order 3, and a knot vector whose spans DIFFER inside the domain.

    WHY A SECOND CASE, and it is not decoration.  `tools/test-bspline-mutations.py` injects the defect
    "take the derivative's knot differences one index early" (`u[p:p+n] - u[0:n]` instead of
    `u[p+1:p+1+n] - u[1:1+n]`) and, on the uniform case ALONE, that defect is INVISIBLE: every span is the
    same number, so the shifted window computes the same values.  The mutation SURVIVED the first version
    of this selftest, and this case -- interior spans 0.4, 0.5, 0.4 s -- is the answer to it.
    """
    order = 3
    pts = np.array([[0.0, 0.0, 0.0], [0.6, 0.5, 0.1], [1.4, 0.9, 0.2],
                    [2.0, 0.6, 0.3], [2.4, -0.1, 0.4], [2.6, -0.6, 0.5]])
    knots = np.array([-1.5, -1.0, -0.5, 0.0, 0.4, 0.9, 1.3, 1.5, 1.7, 2.0])
    return order, pts, knots


def _fd_velocity(knots, pts, order, t, h=1e-5):
    """Velocity by a CENTRAL DIFFERENCE of the position: an independent route to the same number.

    This reaches the arithmetic through `bspline_eval` only, so it tests the analytic derivative spline
    (`_bspline_deriv_ctrl`) against the position evaluator instead of against a re-reading of itself.
    """
    return (bspline_eval(knots, pts, order, t + h) - bspline_eval(knots, pts, order, t - h)) / (2.0 * h)


def _check_case(label, order, pts, knots, interval=None) -> list:
    """Every property the evaluator must have, asserted on ONE case.  Returns the list of problems."""
    n_ctrl = len(pts)
    dur = bspline_duration(knots, n_ctrl, order)
    bad = []
    print("   %s" % label)
    print("      knots %s" % np.round(knots, 3))
    print("      duration %.6f s" % dur)

    # 1. THE DOMAIN, against its textbook definition.  The domain of a degree-p B-spline on the knots
    #    u_0..u_m is [u_p, u_(n+1)], i.e. `u[n_ctrl] - u[order]` here -- the same `lo`/`hi` the Cox-de Boor
    #    reference uses.  `u[m] - u[0]` is the plausible wrong answer, and for the uniform case it is 5.0 s
    #    against the true 0.5 s; a selftest that only evaluates inside [0, dur] cannot see that, because
    #    everything past the real end is clamped to the same point.
    want = float(knots[n_ctrl] - knots[order])
    if abs(dur - want) > 1e-12:
        bad.append("duration %.6f != the domain end u[n_ctrl]-u[order] = %.6f" % (dur, want))
    if interval is not None:                      # the hand-checkable form, uniform knots only
        hand = interval * (n_ctrl - order)
        if abs(dur - hand) > 1e-12:
            bad.append("duration %.6f != interval*(n_ctrl-order) = %.6f" % (dur, hand))

    # 2. THE DERIVATIVE, against a central difference of the position.  Two separate code paths must agree.
    worst_v = 0.0
    for t in np.linspace(0.05 * dur, 0.95 * dur, 7):
        _p, v, _a = bspline_derivs(knots, pts, order, t)
        worst_v = max(worst_v, float(np.max(np.abs(v - _fd_velocity(knots, pts, order, t)))))
    print("      velocity vs a central difference of the position: worst %.3g m/s" % worst_v)
    if worst_v > 1e-5:
        bad.append("the analytic velocity disagrees with a central difference by %.3g m/s" % worst_v)

    # 3. AND THE STRAIGHT-LINE PROPERTY, where the case has one: constant velocity along x, zero elsewhere.
    #    The speed is known by hand and is NOT the control-point spacing: for uniform knots the derivative's
    #    denominator is `p * interval`, so `Q = p * spacing / (p * interval) = spacing / interval`.
    if interval is not None:
        spacing = float(pts[1][0] - pts[0][0])
        want_v = np.array([spacing / interval, 0.0, 0.0])
        worst_line = 0.0
        for t in np.linspace(0.0, dur, 7):
            p, v, a = bspline_derivs(knots, pts, order, t)
            if not np.allclose(v[1:], 0.0, atol=1e-9) or not np.allclose(a, 0.0, atol=1e-9):
                bad.append("t=%.2f: derivative not constant along the line: v=%s a=%s" % (t, v, a))
            worst_line = max(worst_line, float(np.max(np.abs(v - want_v))))
        print("      the line's own speed: worst %.3g m/s from %.3f" % (worst_line, want_v[0]))
        if worst_line > 1e-9:
            bad.append("the straight line does not run at %.3f m/s (worst %.3g)" % (want_v[0], worst_line))

    # 4. THE CLAMP: past the end the evaluator HOLDS the endpoint, it does not extrapolate (ego's own
    #    `evaluateDeBoor` clamps, `uniform_bspline.cpp:63`).
    p_end, _, _ = bspline_derivs(knots, pts, order, dur)
    p_over, _, _ = bspline_derivs(knots, pts, order, dur + 5.0)
    p_under, _, _ = bspline_derivs(knots, pts, order, -5.0)
    held_end = bool(np.allclose(p_end, p_over, atol=1e-9))
    held_start = bool(np.allclose(bspline_eval(knots, pts, order, -5.0),
                                  bspline_eval(knots, pts, order, 0.0), atol=1e-9))
    print("      clamping: past the end %s, before the start %s" % (held_end, held_start))
    if not held_end:
        bad.append("the evaluator extrapolates past the domain instead of clamping: %s vs %s" % (p_end, p_over))
    if not held_start:
        bad.append("the evaluator extrapolates before the domain instead of clamping")

    # 5. THE INDEPENDENT REFERENCE: the same points through the Cox-de Boor basis recursion.
    worst_x = 0.0
    for t in np.linspace(0.0, dur, 9):
        worst_x = max(worst_x, float(np.max(np.abs(bspline_eval(knots, pts, order, t)
                                                      - cox_de_boor(knots, pts, order, t)))))
    print("      de Boor vs Cox-de Boor: worst %.3g m" % worst_x)
    if worst_x > 1e-9:
        bad.append("de Boor and Cox-de Boor disagree by %.3g" % worst_x)
    return bad


def _selftest() -> int:
    """The lifted arithmetic, against TWO cases and three independent references.

    `order, interval, pts, knots = _linear_case()` gives a straight line whose answer is known by hand;
    `_nonuniform_case()` gives a knot vector with unequal interior spans, which is the only way the
    derivative's knot-index arithmetic is pinned down.  See `tools/test-bspline-mutations.py`, which is
    what forced the second case to exist.
    """
    bad = []
    order, interval, pts, knots = _linear_case()
    bad += _check_case("case 1  straight line, uniform knots (the answer is known by hand)",
                       order, pts, knots, interval=interval)
    order2, pts2, knots2 = _nonuniform_case()
    bad += _check_case("case 2  unequal interior spans (pins the derivative's knot arithmetic)",
                       order2, pts2, knots2)
    print()
    if bad:
        print("BSPLINE SELFTEST FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("BSPLINE SELFTEST PASSED: two cases, three references (hand-known line, central difference, "
          "Cox-de Boor)")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
