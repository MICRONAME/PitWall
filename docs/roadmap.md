# MVP roadmap

Implement one small, buildable slice at a time.

| Milestone | Acceptance criterion | Status |
| --- | --- | --- |
| Foundation | Two executables, domain library, CMake presets, offline CTest | Implemented |
| Dependency setup | Verify vcpkg/GoogleTest and commit registry baseline | Pending network access |
| Vehicle model | Fixed-step acceleration, braking, turning; independent tests | Implemented |
| Track and laps | One circuit; ordered checkpoints; directed finish crossing | Planned |
| Interactive simulator | Drive a valid lap using WASD; pause and reset | Planned |
| UDP | Codec validation and loopback integration tests | Planned |
| Dashboard | Live values, speed/pedals charts, source diagnostics | Planned |
| Recording and replay | Save, reopen, pause, seek, change playback speed | Planned |
| MVP release | Windows build, documentation, demo recording, CI | Planned |

After MVP: autopilot, distance-aligned lap comparison, CSV, other tracks and external
simulator adapters. No opponent AI, track editor or detailed drivetrain before MVP.

## Next implementation slice

Add one closed track with centreline geometry, road width and ordered checkpoints.
Implement directed finish crossing and valid/invalid lap tracking independently of
graphics. Then add a window, draw the track/car and connect keyboard controls to
the existing fixed-step runner. Physics must stay usable without a window.

## Risks

- Graphics and package setup can consume the project: keep offline domain builds
  available and add one dependency group at a time.
- Kinematic steering is not a grip model: state its limits and bound control inputs.
- UDP is lossy: expose gaps and incomplete recordings; never silently repair data.
- Rendering stalls must not produce a large integration step.
- Lap comparison needs common track geometry and progress, not raw travelled path.
- Shutdown and backpressure require tests before adding more threads.
