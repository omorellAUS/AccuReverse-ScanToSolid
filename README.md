# AccuReverse-ScanToSolid
Benchmark reverse engineering tool for 3D scans (.stl, .obj, .igs) to simplified meshes (~100-1000 triangles) and watertight solids (.step/.iges) with 0.02mm accuracy (0.01mm buffer). Supports general shapes with automatic detection or hints (e.g., gears, bearings, housings, rotors, figurines). Integrates with Autodesk Fusion 360 via .step/.iges imports for editable solids.

## Features
- Input: .stl, .obj, .igs scans (1-3M triangles).
- Output: Simplified mesh (.stl, 100-1000 triangles following surfaces/edges) and solid B-Rep (.step/.iges).
- Accuracy: Max deviation ≤0.02mm from original, with 0.01mm buffer in fitting/tessellation.
- Shape Options: "any" for auto-detection (analyzes dimensions, curvature for primitives/NURBS), or hints like "gear", "shaft".
- Benchmarking: Tests on diverse shapes, validates Hausdorff distance, compares to baselines.

## Usage
1. Configure `config.json` (set "shape_hint": "any" for general auto-processing based on object dimensions/surfaces).
2. Run: `./build/Release/accureverse config.json tests/gear.stl`.
- For "any" mode: Analyzes overall dimensions (bounding box, aspect ratio) to apply hybrid fitting (primitives for mechanical, NURBS for organic), ensuring 100-1000 triangles aligned to edges/surfaces.
3. Outputs in `output/`: simplified.stl (low-poly mesh), output.step (solid for Fusion 360 import).

## Setup
- Clone: `git clone https://github.com/omorellAUS/AccuReverse-ScanToSolid`
- Install dependencies via vcpkg.
- Build: `cmake -B build -S . -CMAKE_TOOLCHAIN_FILE=./vcpkg/scripts/buildsystems/vcpkg.cmake; cmake --build build`
- Test: `ctest -C Release`

## Benchmarking
Run `benchmarks/run_benchmarks.sh` for accuracy (≤0.02mm), triangle count, runtime on tests/. Results in console/logs.

## Integration with Fusion 360
- Import .step/.iges directly into Fusion 360 as parametric solids for editing (e.g., add features to gears/shafts).
- Accuracy preserved: Solids follow scan dimensions/edges with bounded error.

License: MIT
