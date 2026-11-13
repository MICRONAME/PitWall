# MVP roadmap

Implement one small, buildable slice at a time.

| Milestone | Acceptance criterion | Status |
| --- | --- | --- |
| Foundation | Two executables, domain library, CMake presets, offline CTest | Implemented |
| Dependency setup | Verify vcpkg/GoogleTest and commit registry baseline | Pending network access |
| Vehicle model | Fixed-step acceleration, braking, turning; independent tests | Implemented |
| Track and laps | One circuit; ordered checkpoints; directed finish crossing | Implemented |
| Interactive simulator | Drive a valid lap using WASD; pause and reset | Planned |
| UDP | Codec validation and loopback integration tests | Planned |
| Dashboard | Live values, speed/pedals charts, source diagnostics | Planned |
| Recording and replay | Save, reopen, pause, seek, change playback speed | Planned |
| MVP release | Windows build, documentation, demo recording, CI | Planned |

After MVP: autopilot, distance-aligned lap comparison, CSV, other tracks and external
simulator adapters. No opponent AI, track editor or detailed drivetrain before MVP.

## Next implementation slice

Add a GLFW/OpenGL window, draw the existing road/car and connect WASD to the existing
fixed-step runner. Spawn at Track::pose_at(0) using the track tangent for heading.
Pass every physics tick to LapTracker, not just the final position of a rendered frame.
Add pause/reset and lap status HUD. A paused frame must not produce a duplicate-time
lap sample. Keep domain, simulation and track usable without a window. External
graphics packages will require a working download path or a populated package cache.

## Risks

- Graphics and package setup can consume the project: keep offline domain builds
  available and add one dependency group at a time.
- Kinematic steering is not a grip model: state its limits and bound control inputs.
- UDP is lossy: expose gaps and incomplete recordings; never silently repair data.
- Rendering stalls must not produce a large integration step.
- Lap comparison needs common track geometry and progress, not raw travelled path.
- Shutdown and backpressure require tests before adding more threads.
