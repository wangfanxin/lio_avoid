# The SITL firmware this project flew

ArduPilot **Copter SITL** built from upstream commit `dbe7921` with **one** local change: the truth tap
(`src/patches/apm-0001-sitl-truth-tap.patch`).  Nothing else in the flight code is ours.

| what | value |
|---|---|
| upstream | ArduPilot `dbe792162d06cab66c3475fd5556bf7a120f119e` (`dbe7921`, "Plane: version to 4.7.1") |
| our change | `libraries/SITL/SIM_Aircraft.cpp`, +75 lines, as `src/patches/apm-0001-sitl-truth-tap.patch` |
| patch md5 | `b09ad7ea5ee23c57b13caae1cf22eb55` |
| patch sha256 | `defec3e8f8da3efab03c576ffabba75274a75b8e859970de286496d6214ef09d` |
| binary | `firmware/arducopter-sitl-dbe7921-tap` |
| binary sha256 | `a85352c6ee46495b50b276c2f6f040eacaf8180f4020ef47695aeba117eab368` |
| binary size | 5651896 bytes (5.4M) |
| platform | ELF 64-bit LSB pie executable, x86-64, version 1 (SYSV), dynamically linked, interpreter / |

VERIFIED: `git diff` in the ArduPilot checkout is **byte-identical** to the patch file above, so this binary
and "upstream + patch" are the same firmware.

## Why the whole tree is NOT here

`/home/fanxin/lio_avoid/src/ardupilot` is 1.5 GB (375 MB of `.git`, 111 MB of `build/`).  The reproducible
unit is the PIN plus the PATCH, both of which are in this repository; a binary is added on top so nobody has
to spend the build time to *use* it.

## Build it yourself (the exact recipe)

```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot && git checkout dbe792162d06cab66c3475fd5556bf7a120f119e
git submodule update --init --recursive
/path/to/this/repo/tools/apm-apply-patch.sh .     # applies src/patches/, --check first
./waf configure --board sitl
./waf copter                                      # -> build/sitl/bin/arducopter
```

## Run it

```bash
LIO_TRUTH_TAP=14560 ./arducopter-sitl-dbe7921-tap --model x --speedup 1 -I0 \
    --home -35.363261,149.165230,584,0 --serial0 tcp:5760 --serial1 tcp:5761
```

`LIO_TRUTH_TAP=<udp port>` is the whole runtime interface of the patch: unset, the binary behaves exactly
like upstream (one `getenv` at first use).  With it set, every physics step emits one datagram

    TAP <inst> <t_us> p_ned v_ned q(w,x,y,z body->NED) gyro FRD accel FRD

which `src/sim/truth_rx.py` receives (1200 Hz measured) and fan-outs to the sensor simulator.  `src/tools/run-mission.sh`
starts it exactly as above, with `--serial0` for the BRIDGE and `--serial1` for the SUPERVISOR.

## Licence

ArduPilot is GPLv3.  The corresponding source for this binary is the upstream commit above plus the patch in
`src/patches/` (both public), which is what GPLv3 requires; this file is the written statement of that.
