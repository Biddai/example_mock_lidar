# Mock LiDAR Sensor

A small C++20 project that models a mock LiDAR sensor scanning a 3D voxel occupancy map. The code is intended as a learning-friendly example of separating sensor interfaces from map data, using strongly typed physical units, and validating behavior with GoogleTest.

## What It Does

The project loads a NumPy `.npy` voxel map, reads drone position and heading from an abstract position sensor, and simulates LiDAR beams by ray marching through the map. Each non-zero voxel is treated as occupied space. Scan results report the hit distance and the beam angle relative to the drone, so calling code can reconstruct world-space hit positions without receiving the drone's absolute heading directly from the LiDAR.

## Project Layout

```text
include/cpp_course/      Public interfaces and types
src/                     Library implementation
examples/                Runnable example program
tests/                   GoogleTest unit tests
data_maps/               Small sample occupancy maps
.devcontainer/           Optional VS Code dev container
CMakeLists.txt           CMake build definition
CMakePresets.json        CMake preset for Ninja + vcpkg
vcpkg.json               C++ dependency manifest
```

## Requirements

- C++20 compiler
- CMake 3.20 or newer
- Ninja
- vcpkg

The project dependencies are declared in `vcpkg.json`:

- `mp-units`
- `tinynpy`
- `gtest`

Set `VCPKG_ROOT` before configuring so the CMake preset can find the vcpkg toolchain file.

PowerShell:

```powershell
$env:VCPKG_ROOT = "C:\path\to\vcpkg"
```

Bash:

```bash
export VCPKG_ROOT=/path/to/vcpkg
```

If you use the provided dev container, `VCPKG_ROOT` is set to `/usr/local/vcpkg`.

## Build

Configure and build with the default preset:

```bash
cmake --preset default
cmake --build --preset default
```

Equivalent commands without presets:

```bash
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build
```

On Windows PowerShell, the manual configure command is:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build
```

## Run the Example

After building, run the example against the default sample map:

```bash
./build/mock_lidar_test
```

You can also pass a specific map:

```bash
./build/mock_lidar_test data_maps/five_voxels_y4_pattern.npy
```

On Windows, the executable may be generated as:

```powershell
.\build\mock_lidar_test.exe .\data_maps\five_voxels_y4_pattern.npy
```

## Run Tests

```bash
ctest --test-dir build --output-on-failure
```

You can also run the test executable directly:

```bash
./build/cpp_course_tests
```

## Core Concepts

- `IMap3D` abstracts a 3D occupancy map.
- `VoxelGrid` implements `IMap3D` by loading a row-major 3D `.npy` array with shape `[X, Y, Z]`.
- `IPositionSensor` abstracts the drone position and heading.
- `MockLidarSensor` emits relative LiDAR hits by tracing beams through an `IMap3D`.
- `Units.h` defines strong unit types for X, Y, Z lengths and horizontal/altitude angles using `mp-units`.

Map coordinates are interpreted in centimeters. A position such as `(2 cm, 4 cm, 2 cm)` maps to voxel index `(2, 4, 2)`. Out-of-bounds or negative coordinates are treated as empty space.

## Notes for Publishing

Generated build outputs and installed dependency trees are intentionally ignored by `.gitignore`. Commit the source files, CMake/vcpkg manifests, sample maps, tests, and documentation; leave `build/` and `vcpkg_installed/` out of the repository.

Before publishing publicly, choose a license that matches how you want others to use the code.
