#!/usr/bin/env python3
"""The bridge contract, made ENFORCEABLE: every method the tree calls on a bridge must be declared.

WHY THIS EXISTS.  `bridge/base.py` says "ONE shape, two implementations", and at HEAD `eba1033` that was
a claim about a file rather than a contract that holds: `fsm/mission_fsm.py` called
`send_position_velocity` (`:388`, `:391`), `set_ekf_origin` (`:572`) and `calibrate_accel_simple` (`:582`)
-- none of them declared on `bridge.base.Bridge` -- and reached into implementation privates
(`self.bridge._sent_positions`, `._sent_nav`, `:972-973`) for its own final log.  A contract the only
caller does not obey cannot be checked by reading it, so this test reads the SOURCE and checks it:

  1. **call sites -> declarations.**  Every `self.bridge.<name>` / `<var>.bridge.<name>` attribute the
     tree touches must exist on `bridge.base.Bridge`, be callable, and carry a real body (not
     `NotImplementedError` alone is fine -- a declaration is a declaration; see 3 below).
  2. **declaration -> implementation.**  Every one of those names must exist on
     `bridge.mavlink_impl.MavlinkBridge` too, with a signature that ACCEPTS the way the caller calls it
     (bound with the call site's own argument list).  A declaration with no implementation behind it is
     exactly the `mavros_impl` defect in a smaller shape.
  3. **the negative controls, on a COPY of the tree** (so the proof is not the test asserting itself):
     (a) delete one declaration from the copy's `bridge/base.py` -> this test must FAIL;
     (b) add one call site for a method that does not exist -> this test must FAIL.
     Both are run by `--self-test`, which is how "deleting one declaration must fail the test" is
     proven rather than asserted.  The copy is made with `shutil.copytree` and removed afterwards.

WHAT IT DOES NOT DO: it does not import rclpy, open a socket, or touch a flight controller, and it reads
source text rather than executing the FSM, so it runs offline in well under a second.

usage: tools/test-bridge-contract.py [--root DIR] [--base PATH] [--self-test] [--quiet]
"""
from __future__ import annotations

import argparse
import ast
import inspect
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# Directories that hold no first-party Python we are contracting for: version control, byte-code caches,
# a colcon workspace's build/install trees, and the in-repo LIO fork's vendored sources.
SKIP_DIRS = {".git", "__pycache__", "build", "install", "log", "node_modules", ".venv", "venv",
             "third_party", "frozen"}

# The name of the variable that holds a bridge.  `self.bridge` is how the mission writes it; the
# corner-tour test uses a bare local.  Anything whose LAST attribute segment is `bridge` counts.
BRIDGE_ATTRS = {"bridge"}

# Names that are deliberately NOT part of the transport contract, with the reason.  Empty is the goal:
# an entry here is a hole in the contract, so each one has to say why.
EXEMPT: dict[str, str] = {}

# Files allowed to ASSIGN to a bridge attribute.  A `store` is reported because assigning to the
# transport's own state from a caller is how implementation detail leaks upward -- but a test that
# FABRICATES a fake bridge (`tools/test-corner-tour.py:149` does `bridge.state = lambda ...: _State(pos)`)
# is not a caller writing the transport, it is a stub standing in for one, and there is no other way to
# write it.  The exemption is PER FILE, so the FSM can never acquire it by accident.
FAKE_BRIDGE_FILES = {"tools/test-corner-tour.py", "tools/test-bridge-contract.py",
                     "tools/test-cmd-stream.py"}


def _iter_py(root):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in sorted(filenames):
            if fn.endswith(".py"):
                yield os.path.join(dirpath, fn)


def _is_bridge_expr(node):
    """True for `self.bridge`, `X.bridge`, `a.bridge` -- one attribute whose name is `bridge`."""
    return isinstance(node, ast.Attribute) and node.attr in BRIDGE_ATTRS


def collect_call_sites(root):
    """[(name, kind, file, line, n_args)] for every `<something>.bridge.<name>` in the tree.

    kind is "call", "attr" or "store".  A plain attribute READ is a contract too -- that is exactly how
    the private `_sent_positions`/`_sent_nav` reads were caught -- and a STORE is a hole being dug
    (`self.bridge.x = ...` puts implementation state into the caller), so both are reported with the
    same weight as a call; only a call additionally has its arity bound against the implementation.
    """
    sites = []
    bad = []
    for path in _iter_py(root):
        rel = os.path.relpath(path, root)
        try:
            tree = ast.parse(open(path, encoding="utf-8").read(), filename=path)
        except (SyntaxError, UnicodeDecodeError) as exc:
            bad.append((rel, str(exc)))
            continue
        call_lines = {(n.func.attr, n.lineno)
                      for n in ast.walk(tree)
                      if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                      and _is_bridge_expr(n.func.value)}
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Attribute) and _is_bridge_expr(node.value)):
                continue
            if (node.attr, node.lineno) in call_lines:
                kind = "call"
            elif isinstance(node.ctx, ast.Store):
                kind = "store"
            else:
                kind = "attr"
            sites.append((node.attr, kind, rel, node.lineno))
    return sorted(set(sites)), bad


def _public(name):
    return not name.startswith("_")


def _purge_bridge_modules(keep_root=None):
    """Drop every cached `bridge*` module AND every previously inserted tree from `sys.path`.

    Both halves are load-bearing for the negative controls.  The modules must go because a cached
    `bridge.mavlink_impl` from an earlier control (or from the real tree) would be served instead of the
    copy's; the PATH entries must go because each control copies the tree somewhere new and a stale copy
    still on `sys.path` can win the import.  Measured: without this the "remove an implementation" control
    read a previous control's copy, where `counters()` still existed, and reported 0 problems while
    claiming to be a passing control.
    """
    for name in [n for n in sys.modules if n == "bridge" or n.startswith("bridge.")]:
        sys.modules.pop(name, None)
    if keep_root is not None:
        # the tree under test is about to be inserted explicitly; drop stale copies of it
        sys.path[:] = [p for p in sys.path if p != keep_root]


def load_base(base_path, root=None):
    """Import `bridge.base` FROM `base_path`, without leaving a stale copy in `sys.modules`.

    Clearing matters for the negative controls: the mutated tree is a copy, so `bridge.base` /
    `bridge.mavlink_impl` must be re-imported from that copy rather than served from the cache of the
    tree this process started in -- otherwise the control would pass while proving nothing.  The module
    is ALSO registered in `sys.modules` before execution, because `@dataclass` looks its own class's
    module up there (`dataclasses._is_type` -> `sys.modules[cls.__module__].__dict__`) and raises
    `AttributeError: 'NoneType' object has no attribute '__dict__'` if it is missing -- measured, not
    guessed: that is what the first version of this loader did.
    """
    import importlib.util
    _purge_bridge_modules(root)
    spec = importlib.util.spec_from_file_location("bridge_base_under_test", base_path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["bridge_base_under_test"] = mod
    spec.loader.exec_module(mod)
    return mod


def load_impl(root):
    """Import `bridge.mavlink_impl` from `root` without needing pymavlink installed."""
    _purge_bridge_modules(root)
    sys.path.insert(0, root)
    import importlib
    return importlib.import_module("bridge.mavlink_impl")


def check(root, base_path=None, quiet=False):
    """Returns (problems, lines).  `problems` empty == contract holds."""
    problems, lines = [], []
    root = os.path.abspath(root)
    base_path = base_path or os.path.join(root, "bridge", "base.py")
    base = load_base(base_path, root)
    Bridge = base.Bridge
    sites, bad = collect_call_sites(root)

    lines.append("bridge contract: %d call/attribute site(s) over %s"
                 % (len(sites), root))
    for rel, exc in bad:
        lines.append("   (skipped: %s does not parse: %s)" % (rel, exc))

    # ---- 1. every call site resolves on the base class --------------------------------------------
    seen = {}
    for name, kind, rel, lineno in sites:
        if name in EXEMPT:
            continue
        seen.setdefault(name, []).append((rel, lineno, kind))
        attr = getattr(Bridge, name, None)
        where = "%s:%d" % (rel, lineno)
        if attr is None:
            problems.append("%s uses bridge.%s, which bridge/base.py does not declare" % (where, name))
            continue
        if kind == "call" and not callable(attr):
            problems.append("%s calls bridge.%s, which is not callable on Bridge" % (where, name))
            continue
        if not _public(name):
            problems.append("%s uses bridge.%s, a PRIVATE name: implementation privates are not a "
                            "contract" % (where, name))
        if kind == "store" and rel not in FAKE_BRIDGE_FILES:
            problems.append("%s ASSIGNS bridge.%s: the caller is writing the transport's own state"
                            % (where, name))
    lines.append("   names the tree uses: %s"
                 % ", ".join("%s x%d" % (n, len(v)) for n, v in sorted(seen.items())))

    # the base class must not have grown names nothing uses: a declaration with no caller is how a
    # second implementation gets invented that nothing can check
    declared = {n for n, v in vars(Bridge).items() if callable(v) and not n.startswith("_")}
    lines.append("   declared on Bridge but unused by the tree: %s"
                 % (", ".join(sorted(declared - set(seen))) or "(none)"))

    # ---- 2. declaration -> implementation, with the caller's own argument shape --------------------
    try:
        impl = load_impl(root)
        Mavlink = impl.MavlinkBridge
    except Exception as exc:                             # pragma: no cover - reported, not raised
        problems.append("cannot import bridge.mavlink_impl: %r" % (exc,))
        lines.append("   [FAIL] bridge.mavlink_impl did not import: %r" % (exc,))
        return problems, lines

    for name, uses in sorted(seen.items()):
        base_attr = getattr(Bridge, name, None)
        # `MavlinkBridge.counters` must be MavlinkBridge's OWN, and `getattr` cannot tell: an inherited
        # `Bridge.counters` (which only raises NotImplementedError) looks exactly like an override.
        # Measured: the "remove an implementation" control below passed against `getattr` and only began
        # failing when this read the class's own `__dict__`.  That is the whole class of defect this test
        # exists for -- a declaration that is not an implementation.
        impl_attr = vars(Mavlink).get(name)
        if impl_attr is None:
            problems.append("bridge/base.py declares %s() but MavlinkBridge does not implement it "
                            "(any attribute it has is inherited from Bridge, i.e. the declaration "
                            "itself)" % name)
            continue
        if callable(base_attr) and callable(impl_attr):
            b_sig = inspect.signature(base_attr)
            i_sig = inspect.signature(impl_attr)
            if list(b_sig.parameters) != list(i_sig.parameters):
                problems.append("signature mismatch for %s: Bridge%s vs MavlinkBridge%s"
                                % (name, b_sig, i_sig))
            b_req = [p for p in b_sig.parameters if b_sig.parameters[p].default is inspect.Parameter.empty]
            i_req = [p for p in i_sig.parameters if i_sig.parameters[p].default is inspect.Parameter.empty]
            if b_req != i_req:
                problems.append("required-argument mismatch for %s: Bridge requires %s, MavlinkBridge %s"
                                % (name, b_req, i_req))
        # every CALL SITE must be plausible against the implementation it will actually reach
        for rel, lineno, kind in uses:
            if kind != "call" or not callable(impl_attr):
                continue
            req = [p for p in inspect.signature(impl_attr).parameters.values()
                   if p.default is inspect.Parameter.empty and p.kind in
                   (inspect.Parameter.POSITIONAL_ONLY, inspect.Parameter.POSITIONAL_OR_KEYWORD)]
            params = inspect.signature(impl_attr).parameters
            # the call `self.bridge.f(a, b)` reaches a bound method, so `self` is already supplied:
            # the number of parameters the IMPLEMENTATION requires must be <= 1 (self) + the caller's
            # positional count.  `mission_fsm.py:391` calls `send_position_velocity(cmd, vel, yaw)`
            # while `send_position_velocity` declares `a_f=None`, which is why the default is allowed.
            if len(req) - 1 > _call_arity(rel, lineno, root):
                problems.append("%s:%d calls bridge.%s with too few arguments: MavlinkBridge requires %s"
                                % (rel, lineno, name, [p for p in req if p != "self"]))
            if not any(p.kind is inspect.Parameter.VAR_POSITIONAL for p in params.values()) and \
                    _call_arity(rel, lineno, root) > len(params) - 1:
                problems.append("%s:%d calls bridge.%s with more arguments than MavlinkBridge accepts "
                                "(%s)" % (rel, lineno, name, list(params)))
    lines.append("   implementations present on MavlinkBridge: %d of %d names the tree uses"
                 % (sum(1 for n in seen if vars(Mavlink).get(n) is not None), len(seen)))

    # ... and the reverse direction: every declaration the implementation OVERRIDES must be declared on
    # the base class, and every method the tree CALLS must be overridden (not merely inherited -- see the
    # `vars(Mavlink)` note above).  The two directions together are what make "ONE shape" checkable.
    # `Bridge` legitimately declares names that no implementation overrides yet (`disarm`,
    # `calibrate_gyros`), so the reverse check is driven by the call sites, not by the whole list.
    undeclared_overrides = sorted(n for n, v in vars(Mavlink).items()
                                  if callable(v) and not n.startswith("_") and not hasattr(Bridge, n))
    for n in undeclared_overrides:
        problems.append("MavlinkBridge implements %s(), which bridge/base.py does not declare" % n)
    lines.append("   MavlinkBridge methods not declared on Bridge: %s"
                 % (", ".join(undeclared_overrides) or "(none)"))
    return problems, lines


def _call_arity(rel, lineno, root):
    """How many positional arguments the call at `rel:lineno` passes (recomputed, so it is exact)."""
    path = os.path.join(root, rel)
    tree = ast.parse(open(path, encoding="utf-8").read(), filename=path)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and _is_bridge_expr(node.func.value) and node.lineno == lineno:
            return len(node.args)
    return 0


def run(root, base_path=None, quiet=False):
    problems, lines = check(root, base_path=base_path)
    if not quiet:
        for line in lines:
            print(line)
        for p in problems:
            print("   [FAIL] %s" % p)
    return problems


def _mutate(path, old, new, required=True):
    """Apply one textual mutation, and REFUSE to run a control whose anchor has drifted.

    `required=True` is the point: a negative control that silently stops matching is a control that
    silently stops controlling (this is exactly what happened to the first version of the "add a call
    site" control -- its anchor had 8 leading spaces while the source line has 12, so the copy was never
    mutated and the control "passed" while proving nothing).  `--self-test` therefore fails loudly
    instead of reporting a green run.
    """
    src = open(path, encoding="utf-8").read()
    if required and old not in src:
        raise SystemExit("negative control: %r not found in %s -- the control is stale, fix the test"
                         % (old[:60], path))
    open(path, "w", encoding="utf-8").write(src.replace(old, new, 1))


def _fingerprint(root):
    """Total size of the copy's first-party Python -- proof the mutation actually landed on disk."""
    return sum(os.path.getsize(p) for p in _iter_py(root) if "bridge" in p or "fsm" in p or "tools" in p)


def self_test(root):
    """The negative controls, each on its OWN copy of the tree."""
    ok = True
    tmp = tempfile.mkdtemp(prefix="bridge-contract-neg-")
    try:
        for label, mutate, must in (
            ("delete a declaration (send_position_velocity) from bridge/base.py",
             lambda r: _mutate(os.path.join(r, "bridge", "base.py"),
                               "    def send_position_velocity", "    def _removed_by_negative_control"),
             "declare"),
            ("add a call site for a method Bridge does not declare",
             lambda r: _mutate(os.path.join(r, "fsm", "mission_fsm.py"),
                               "            self.bridge.poll()",
                               "            self.bridge.poll()\n"
                               "            self.bridge.teleport_home()"),
             "does not declare"),
            ("remove an implementation from bridge/mavlink_impl.py",
             lambda r: _mutate(os.path.join(r, "bridge", "mavlink_impl.py"),
                               "    def counters(self):", "    def _counters_removed(self):"),
             "does not implement"),
        ):
            dst = os.path.join(tmp, "copy%d" % len(os.listdir(tmp)))
            shutil.copytree(root, dst, ignore=shutil.ignore_patterns(*SKIP_DIRS))
            before = _fingerprint(dst)
            mutate(dst)
            after = _fingerprint(dst)
            print("NEGATIVE CONTROL: %s" % label)
            if before == after:
                print("   the COPY was not modified -- the control is stale (anchor drifted?)")
                ok = False
                shutil.rmtree(dst, ignore_errors=True)
                continue
            problems = run(dst, quiet=True)
            hit = [p for p in problems if must in p]
            print("   the copy's Python changed (%d -> %d bytes of source); the test reported %d problem(s)"
                  % (before, after, len(problems)))
            print("   verdict: %s" % ("FAILS as required" if hit else "DID NOT FAIL -- the control is dead"))
            for p in problems[:3]:
                print("      %s" % p)
            if not hit:
                ok = False
            shutil.rmtree(dst, ignore_errors=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=ROOT, help="the tree to read (default: this repository)")
    ap.add_argument("--base", default=None, help="bridge/base.py to check against (default: <root>'s)")
    ap.add_argument("--self-test", action="store_true", help="run the negative controls on copies")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    problems = run(args.root, base_path=args.base, quiet=args.quiet)
    ok = not problems
    if args.self_test:
        ok = self_test(os.path.abspath(args.root)) and ok
    print()
    print("BRIDGE-CONTRACT TEST %s" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
