# Telemetry contract

## Implemented domain values

`TelemetrySample` is an in-memory type. It is not a serialized packet. Validation
returns the first field error. It does not enforce packet order or session changes.

| Field | Units / meaning | Current validation |
| --- | --- | --- |
| session_id | Unsigned 64-bit session token | Nonzero |
| sequence | Unsigned 64-bit sample index | Zero allowed |
| session_time | Microseconds since simulation start | Nonnegative |
| position_m.x / y | Metres in track coordinates | Finite; negative allowed |
| speed_mps | Metres/second, forward speed | Finite, nonnegative |
| longitudinal_acceleration_mps2 | Metres/second squared | Finite; negative allowed |
| heading_rad | Radians, mathematical orientation | Finite; not normalized yet |
| steering_rad | Radians | Finite; vehicle limit is a simulation concern |
| throttle / brake | Normalized pedal input | Finite, inclusive [0, 1] |

Both pedals may be applied simultaneously. Exact vehicle limits belong to vehicle
configuration rather than a universal telemetry validator. Reverse is outside MVP.

## Planned wire protocol (not implemented)

Initial destination: 127.0.0.1:5000, 50 Hz, one UTF-8 JSON object per datagram.
Add an explicit version and message kind. Define exact integer encodings before
implementation; do not lose 64-bit IDs through floating-point JSON consumers.
Keep datagrams below a defined size limit and document track/session metadata.

Messages will include samples and source status (running/paused). Pause stops the
simulation clock but continues status messages. A reset creates a new session token.
Publish session/track metadata repeatedly so an Analyzer started late can recover.

UDP has no connection handshake. UI status is active/paused/no data based on last
valid messages and local monotonic receive time. Unknown versions, malformed fields,
oversized messages, duplicates and reordered samples need separate handling.
Sequence gaps are estimated missing samples, not proof of network-only packet loss.
Do not claim reliable recording: gaps must remain visible in saved sessions.

Lap metadata, centreline progress and status messages are deferred until track and
session rules exist. RPM/gear, if introduced synthetically, must be labelled as such.
