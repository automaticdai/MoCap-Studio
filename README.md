# MoCap Studio

Real-time, markerless human motion capture from multiple synchronised cameras.

**Pipeline:** Capture → 2D Pose Estimation → 3D Triangulation → Skeleton Solving → Export

## Dependencies

### Required (install manually)

| Library | Version | Purpose |
|---------|---------|---------|
| Qt6 | 6.5+ | GUI (Widgets + OpenGL) |
| OpenCV | 4.8+ | Camera I/O, calibration |
| ONNX Runtime | 1.16+ | Pose model inference |

### Auto-fetched by CMake

These are downloaded automatically via FetchContent if not found on the system:

| Library | Purpose |
|---------|---------|
| Eigen 3.4 | Linear algebra |
| spdlog 1.12+ | Logging |
| yaml-cpp 0.7+ | Config parsing |
| nlohmann/json 3.11+ | JSON serialisation |
| ezc3d | C3D file I/O |
| Google Test 1.14 | Unit testing |

### Optional

| Library | Purpose |
|---------|---------|
| Assimp | FBX export (enabled automatically if found) |
| Blackmagic DeckLink SDK | DeckLink capture cards |

## Installation

### Ubuntu 22.04+

```bash
# Required
sudo apt install \
  qt6-base-dev libqt6opengl6-dev libqt6openglwidgets6 \
  libopencv-dev \
  libeigen3-dev libspdlog-dev libyaml-cpp-dev

# Optional (for FBX export)
sudo apt install libassimp-dev
```

**ONNX Runtime** must be installed manually:

```bash
# Download from https://github.com/microsoft/onnxruntime/releases
# Example for Linux x64 with CUDA:
wget https://github.com/microsoft/onnxruntime/releases/download/v1.17.0/onnxruntime-linux-x64-gpu-1.17.0.tgz
tar xzf onnxruntime-linux-x64-gpu-1.17.0.tgz
export ONNXRUNTIME_ROOT=$(pwd)/onnxruntime-linux-x64-gpu-1.17.0
```

### Windows (MSVC 2022)

1. Install Qt6 via the [Qt Online Installer](https://www.qt.io/download-qt-installer)
2. Install OpenCV via [vcpkg](https://github.com/microsoft/vcpkg): `vcpkg install opencv4`
3. Download ONNX Runtime from [GitHub releases](https://github.com/microsoft/onnxruntime/releases)
4. Set environment variables: `Qt6_DIR`, `OpenCV_DIR`, `ONNXRUNTIME_ROOT`

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel
```

### Custom dependency paths

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DQt6_DIR=/path/to/qt6/lib/cmake/Qt6 \
  -DOpenCV_DIR=/path/to/opencv/lib/cmake/opencv4 \
  -DONNXRUNTIME_ROOT=/path/to/onnxruntime
```

### CMake options

| Option | Default | Description |
|--------|---------|-------------|
| `MOCAP_BUILD_TESTS` | `ON` | Build unit tests |
| `MOCAP_ENABLE_BLACKMAGIC` | `OFF` | Enable Blackmagic DeckLink support |

## Run

```bash
# Uses config.yaml from current directory
./build/MoCapStudio

# Custom config path
./build/MoCapStudio /path/to/config.yaml
```

## Tests

```bash
cd build
ctest --output-on-failure
```

## Configuration

Edit `config.yaml` to configure cameras, pose estimation, triangulation, and skeleton settings. Example:

```yaml
capture:
  target_fps: 60.0
  max_sync_skew_ms: 5.0

cameras:
  - id: cam0
    type: usb
    device_index: 0
    resolution: [1920, 1080]
  - id: cam1
    type: ip
    url: "rtsp://192.168.1.100:554/stream"
    resolution: [1920, 1080]

pose_estimation:
  model: "models/rtmpose_l.onnx"
  device: "cuda"

triangulation:
  min_views: 2
  ransac_enabled: true
  ransac_threshold_px: 5.0
  filter_cutoff_hz: 6.0

skeleton:
  definition: "body_25"
  ik_solver: "analytical"
  joint_limits_enabled: true
```

## Export Formats

| Format | Data Layers | Use Case |
|--------|-------------|----------|
| CSV | L1, L2, L3 | Spreadsheet analysis |
| JSON | L1, L2, L3 | Web/programmatic access |
| C3D | L2 (3D markers) | Biomechanics software |
| BVH | L3 (skeleton) | Animation software |
| FBX | L3 (skeleton) | Game engines, DCC tools |
| USD | L3 (skeleton) | Film/VFX pipelines |

## Project Structure

```
src/
├── core/           # Data types, config, skeleton definition
├── capture/        # Camera sources, frame broker
├── pose/           # 2D pose estimation, person tracking
├── triangulation/  # 3D lifting, temporal filtering
├── skeleton/       # IK solver
├── storage/        # Session manager, binary I/O, exporters
├── gui/            # Qt widgets (canvas, timeline, inspector, dialogs)
└── main.cpp
tests/              # Google Test unit tests
resources/          # Shaders, skeleton definitions
cmake/              # Custom find modules
```

## License

TBD
