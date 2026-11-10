# PitWall

Racing Simulator & Telemetry Analyzer, written in C++20.

**Status: Vehicle Physics (0.2.0).** The simulator now has a GUI-independent bicycle
model, longitudinal forces, steering and speed limits, a 100 Hz fixed-step runner,
and a scripted headless demo. It is not yet a playable game. Graphics, tracks,
keyboard driving, UDP, SQLite, replay and lap analysis are planned, not implemented.

## Build on Windows: available MinGW toolchain

Requirements: CMake 3.24+, a C++20 compiler, and `mingw32-make` on PATH.
For MSYS2 UCRT64, add its `ucrt64/bin` directory to PATH. All build commands below
run from the directory containing this README.

```powershell
cmake --preset windows-mingw
cmake --build --preset windows-mingw --parallel 2
ctest --preset windows-mingw
.\build\windows-mingw\apps\simulator\pitwall_simulator.exe
.\build\windows-mingw\apps\simulator\pitwall_simulator.exe --demo
.\build\windows-mingw\apps\analyzer\pitwall_analyzer.exe
```

Both programs also accept `--help` and `--version`. Unknown arguments return exit
code 2. Normal startup prints status and exits. Simulator `--demo` computes 12
seconds of acceleration, left turning and braking, printing one row per simulated
second. It runs without sleeping and opens no window. It is a scripted input sequence,
not an autopilot. Analyzer is still the foundation entry point.

## Visual Studio 2022

Install the Desktop development with C++ workload, then:

```powershell
cmake --preset windows-msvc
cmake --build --preset windows-msvc
ctest --preset windows-msvc
.\build\windows-msvc\apps\simulator\Debug\pitwall_simulator.exe
.\build\windows-msvc\apps\analyzer\Debug\pitwall_analyzer.exe
```

## GoogleTest via vcpkg

The default foundation build has no external dependencies. Its small offline test
runner executes the same domain contract as the optional GoogleTest runner.
To enable GoogleTest, install/bootstrap vcpkg separately and set `VCPKG_ROOT`:

```powershell
$env:VCPKG_ROOT = 'C:\dev\vcpkg'
cmake --preset windows-msvc-vcpkg
cmake --build --preset windows-msvc-vcpkg
ctest --preset windows-msvc-vcpkg
```

The manifest currently requests only `gtest`. Add graphics, network and storage
packages when their implementations are introduced. A registry baseline is not yet
pinned: dependency reproducibility is a pending foundation task. Once vcpkg is
available, run `vcpkg x-update-baseline --add-initial-baseline` from this directory
and commit the resulting `builtin-baseline`. External dependencies require network
access or a prepopulated package cache.

## Linux

```sh
cmake --preset linux-debug
cmake --build --preset linux-debug --parallel 2
ctest --preset linux-debug
./build/linux-debug/apps/simulator/pitwall_simulator
./build/linux-debug/apps/analyzer/pitwall_analyzer
```

Linux and GoogleTest configurations have not been locally verified yet.

## Architecture

See [architecture](docs/architecture.md), [telemetry contract](docs/telemetry.md),
[roadmap](docs/roadmap.md) and [verification](docs/verification.md).

Current source layout:

- `apps/simulator`: simulator entry point.
- `apps/analyzer`: analyzer entry point.
- `libs/domain`: shared telemetry types and stateless field validation.
- `libs/simulation`: vehicle parameters, physics and a bounded fixed-step accumulator.
- `tests`: domain, physics, frame scheduling and application checks.
- `docs`: design decisions, planned milestones and verification evidence.

No empty module libraries or placeholder interfaces are created for future features.
Windows is the primary target; domain code uses only standard C++20.

## Development

Use `.clang-format` for source formatting. Avoid serializing C++ object memory;
the future wire codec must define an independent format. Keep UI and third-party
dependencies out of the domain library. Build and test after each small milestone.

Suggested milestone commit: `feat: add fixed-step vehicle physics and headless demo`.

License: MIT; see [LICENSE](LICENSE).
