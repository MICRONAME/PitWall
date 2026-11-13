# Verification

Verified locally on Windows on 2026-11-10:

| Configuration | Compiler | Result |
| --- | --- | --- |
| windows-mingw, Debug | GNU C++ 16.2.0 / MSYS2 UCRT64 | Configured and built; 58/58 CTest tests passed |
| windows-msvc, Debug | MSVC 19.41.34123.0 / Visual Studio 2022 | Configured and built; 58/58 CTest tests passed |

CMake version: 4.4.0-rc1. Builds produced no compiler warnings.

The domain contract checks valid defaults, session IDs, negative simulation time,
negative speed, pedal limits, valid negative coordinates/deceleration and NaN / both
infinities in every floating-point field. Checks remain active in Release builds.
Eight tests exercise startup, help, version and unknown arguments for both
applications. Twenty named physics tests verify rest, analytic acceleration and
stopping distance (including a stop inside a tick), forces, speed clamp, turn symmetry,
analytic circular motion, steering slew, curvature cap, invalid inputs/config/state,
frame scheduling at 30 vs 144 FPS, remainder preservation, overload, reset, telemetry,
atomic rejection and a 100-second turning run. One integration test runs the scripted
demo and requires completion after 1200 physics ticks.

The demo was also run manually: after 12 simulated seconds its final speed is 0 km/h,
travelled distance approximately 172.550 m and position approximately (148.989, 63.316) m.
It emits valid domain samples checked at each printed second.

Twenty-seven track tests cover perimeter, wrapping and projection, road boundaries,
swept corridor containment, rounded corners, invalid geometry/rules/samples, default
oval geometry, directed start, checkpoint order, lap time, best valid time, off-track
and shortcut invalidation, discontinuities, reverse crossings, exact line endpoints,
missing checkpoints, recovery, reset, interpolation, multiple crossings in one sample
and large timestamps. The final track test feeds the actual VehicleModel around a
30 m radius circular test track; it completes two valid laps with times matching the
analytic circle period within 0.1 ms.

An additional application test runs `--track-demo`: default oval length 400.274 m,
two valid laps at about 20.014 s each. This demo feeds scripted positions rather than
using a vehicle controller. Both existing physics demo and new track demo were run.

Not verified: Linux, the vcpkg/GoogleTest preset, or GitHub Actions execution.
GitHub access from the local execution environment failed with a DNS/thread error,
so external dependencies were not downloaded. No dependency baseline is claimed.
The GitHub Actions workflow is provided for future repository pushes.

Graphics, keyboard driving, network, database and replay are not implemented.
