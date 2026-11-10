# Vehicle physics: implemented in 0.2.0

## Responsibilities

- `VehicleConfig`: mass, wheelbase, forces and limits; supplied by value at construction.
- `ControlInput`: normalized gas/brake [0,1] and steering [-1,1]. Invalid input is
  rejected rather than silently clamped. Positive steering is a left turn.
- `VehicleState`: position of the rear axle, forward speed, heading, applied steering,
  realised longitudinal acceleration and actual travelled distance.
- `VehicleModel::step`: returns a new state with no IO or modification of its argument.
- `FixedStepSimulation`: owns state and a 10 ms accumulator, clock and tick count.

All quantities use SI units. +X points along heading zero; +Y points upwards.
Angles are radians. Heading is normalized to [-pi, pi]. This is not screen space.
The track renderer will convert world coordinates to pixels.

## Longitudinal motion

For each tick, evaluate forces using its initial speed:

```
F = throttle * engine_force - brake * brake_force - rolling_resistance - drag * v^2
a = F / mass
v_next = clamp(v + a * dt, 0, max_speed)
```

Acceleration is held constant within a tick. If speed reaches zero or the cap
mid-tick, integrate motion up to that event, then keep the clamped speed for the
remaining time. This preserves the correct constant-acceleration stopping distance
and avoids moving backwards or creeping under the brakes at rest.

Telemetry acceleration is `(v_next - v) / dt`, including clamping effects.
Rolling resistance acts as a constant forward-opposing force; the nonnegative
speed constraint approximates static rest. There is no reverse gear or drivetrain.

## Steering and trajectory

Requested wheel angle is `steering * max_steering_rad`; approach it using the
configured steering rate. Curvature is `tan(wheel_angle) / wheelbase`.
At nonzero speed, bound its magnitude by `max_lateral_acceleration / peak_speed^2`,
using the larger speed at the tick's endpoints. Reconstruct the applied wheel angle
from the bounded curvature. This safety correction can override steering slew when
needed; it is a geometric approximation, not tire slip or tire-force dynamics.

The bicycle equations are `dx/dt = v*cos(heading)`, `dy/dt = v*sin(heading)` and
`dheading/dt = v*tan(wheel_angle)/wheelbase`. We integrate a constant-curvature
circular arc using the travelled distance. A sinc expression avoids subtracting
nearly equal numbers at small turn angles. Straight motion is its zero-angle limit.

Primary references:
- https://petercorke.github.io/robotics-toolbox-python/mobile-vehicle-bicycle.html
- https://gafferongames.com/post/fix_your_timestep/

## Fixed-step scheduling

`advance(frame_time, controls)` accepts [0,60s] per call. It accumulates integer
microseconds, executes 10 ms ticks and preserves the fractional remainder. Default
maximum: 8 ticks per call, configurable in [1,1000]. Excess whole ticks are discarded
and returned as `FrameResult::dropped_time`. Simulation time advances only for
executed ticks. A slow frame never causes one large physics integration step.

`interpolation_alpha` is remainder/10ms for future rendering; interpolation itself
is not implemented. Input is held for ticks executed by the call. Identical inputs
at physics ticks produce identical results in the same build, independent of frame
partitioning if there is no overload. Different frame-based input sampling can change
the effective control history; no cross-platform bitwise determinism is promised.

State is committed only after an entire advance succeeds. Invalid controls (even
on zero-tick frames), invalid durations, states/configurations and arithmetic overflow
are reported through exceptions. `reset()` clears state, input, remainder and clock.
The caller assigns session IDs; reset does not invent a unique ID.

`telemetry(session_id)` converts the current physics state to a domain sample.
Sequence is physics tick count, NOT a future UDP packet counter; a sender must use
its own sequence if transmitting at 50 Hz. Last controls refer to executed ticks.
No network transmission occurs in this module.

## Demo

```
pitwall_simulator --demo
```

The demo computes 12 simulated seconds (1200 ticks), prints a table and exits:
0-4s full throttle, 4-7s partial throttle with left steering, 7-12s full braking.
It does not sleep, open a window, follow a track or accept keyboard control.

## Model limits

Illustrative parameters, no calibrated real vehicle; no lateral tire dynamics,
weight transfer, suspension, collisions, slopes, aerodynamic downforce, gears or RPM.
Distance travelled is distinct from future centreline progress for lap comparison.
