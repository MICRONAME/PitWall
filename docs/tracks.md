# Track geometry and lap timing (0.3.0)

## Implemented scope

An independent `pitwall::track` library with immutable closed geometry and stateful
lap timing. The app still runs headlessly. There are no graphics, collisions, track
file parser, track editor or autopilot. Geometry is supplied as ordinary C++ values.

## Geometry contract

`Track(id, vertices, road_width_m)` takes 3..4096 vertices without a repeated closing
point. The last-to-first segment is implicit. All coordinates use metres and must be
finite within +/-1,000,000 m. Segment length is at least 0.00001 m and road width is
in [0.0001,10000] m. Reject empty IDs, zero-length edges, adjacent backtracking and
self-intersecting/touching nonadjacent centreline edges. The direction is vertex order.

The road is a union of constant-radius segment capsules: rectangular strips with
rounded endpoint disks. It includes its boundary (a 1e-9 m numerical tolerance).
This supports simple non-overlapping circuits. Nonadjacent road corridors can still
overlap for overly wide roads; such layouts are outside the intended MVP geometry
and can make nearest-segment progress ambiguous. Centreline validation alone does
not prove the full road corridor is non-overlapping.

Start/finish is at the midpoint of the first segment, so its direction is unambiguous.
Its finite gate spans the road width, perpendicular to that segment. `pose_at(s)`
wraps finite positive or negative distances to [0,length), with zero at this start.
`project(position)` returns the nearest centreline position, tangent, wrapped
progress, segment index, distance and on-road flag. Ties select the first segment.
Signed lateral offset is nearest-point distance signed by the selected tangent's
cross product, so at a vertex it is not a separate Frenet-coordinate solution.
It is not the vehicle's accumulated travelled distance or an unwrapped lap distance.

`contains_motion(from,to)` checks the ENTIRE straight sampled motion segment. Each
road rectangle/disk covers an interval of motion parameter t in [0,1]; the sorted
union must cover all of [0,1]. Checking endpoints alone would miss infield shortcuts.
The approximation is the straight chord between samples, not the exact continuous
vehicle arc. Feed positions at physics ticks for sufficiently fine sampling.

Default oval: 64-point ellipse with radii 80 and 45 m, road width 10 m and polyline
length about 400.274 m. No generated raster assets or third-party geometry are used.

## Timing contract

`LapTracker` owns a Track value. Defaults: checkpoints at 1/4, 1/2 and 3/4 length;
maximum displacement between samples 10 m. Override these through its constructor.
Custom checkpoints must be strictly increasing inside (0,length). Empty means defaults.

`update(position, simulation_time)` requires finite bounded coordinates and nonnegative,
strictly increasing microsecond timestamps after its first sample. Invalid input
throws without changing state. Time is simulation time, not wall-clock receive time.
A pause should produce no update until simulation time advances.

The first sample only initializes history. A forward start crossing begins lap 1.
Finite gates are checked against the entire displacement segment; all crossings are
processed in ascending movement fraction, including multiple gates inside one sample.
An exact-on-line point belongs to the forward side: a forward arrival counts once,
forward departure does not repeat, and a backwards departure is detected.
Crossing times are linearly interpolated between the two timestamps (truncated to
microseconds). Equal timestamps are rejected, not silently used for zero-time laps.

Lap validity requires all checkpoints in order before a forward finish crossing.
Off-road motion, a shortcut, excessive displacement, reverse gate crossing or an
unexpected checkpoint invalidates the current lap. The first reason is preserved.
Bad movements cannot themselves start or finish a lap. Invalidity persists after
returning to the road. A later legal forward finish records the invalid lap and
begins a clean one. Best time includes only complete valid laps. Invalid laps remain
in history. `reset()` clears history, best time, clock and checkpoint state.

The displacement limit is a gap/teleport safeguard, not a proof of physical continuity
or a speed limit. Motion at sub-sample resolution cannot be reconstructed. No hostile
telemetry/anti-cheat guarantee is claimed. An overlapping-corridor circuit is unsupported.

The tracker checks a point (vehicle rear axle), not every wheel/body corner. Vehicle
width, track collisions and penalties are future simulation/gameplay concerns.
Lap result/status are separate from TelemetrySample for now; UDP is not implemented.

## Demos and tests

`pitwall_simulator --track-demo` feeds scripted centreline positions at 20 m/s and
prints two laps. This is a geometry/timing demo, not an autopilot or physical drive.
An independent integration test passes actual VehicleModel positions around a circle
to LapTracker and checks valid lap counts and the analytic period.

`ctest --preset windows-mingw -R track` runs track/lap tests and the track demo.

## Main source files

- `libs/track/include/pitwall/track/track.hpp`: immutable geometry API and projection types.
- `libs/track/src/track.cpp`: validation, length, projection and swept-road containment.
- `libs/track/include/pitwall/track/lap_tracker.hpp`: lap API, results and validity reasons.
- `libs/track/src/lap_tracker.cpp`: ordered gates, crossing interpolation and timing.
- `tests/track_cases.hpp`: shared named cases for offline and GoogleTest runners.
