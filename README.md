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

### Auto-fetched by Meson

These are downloaded automatically as Meson subprojects (wrap files under
`subprojects/`) if not found on the system:

| Library | Purpose |
|---------|---------|
| Eigen 3.4+ | Linear algebra |
| spdlog 1.x | Logging (always built from the subproject, with a pinned `fmt`) |
| yaml-cpp 0.7+ | Config parsing |
| nlohmann/json 3.11+ | JSON serialisation |
| ezc3d | C3D file I/O (built via its CMake through Meson's `cmake` module) |
| Google Test 1.x | Unit testing |

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
  meson ninja-build \
  qt6-base-dev libqt6opengl6-dev libqt6openglwidgets6 \
  libopencv-dev \
  libeigen3-dev libyaml-cpp-dev

# Optional (for FBX export)
sudo apt install libassimp-dev
```

**ONNX Runtime** must be installed manually:

```bash
# Download from https://github.com/microsoft/onnxruntime/releases
# Example for Linux x64 with CUDA:
wget https://github.com/microsoft/onnxruntime/releases/download/v1.17.0/onnxruntime-linux-x64-gpu-1.17.0.tgz
sudo tar xzf onnxruntime-linux-x64-gpu-1.17.0.tgz -C /opt
# Anything matching /opt/onnxruntime* or /usr/local/onnxruntime* is auto-discovered.
# For any other location, pass -Donnxruntime_root=/path/to/onnxruntime at setup.
```

### Windows (MSVC 2022)

1. Install Meson + Ninja: `pip install meson ninja` (or `pipx install meson`)
2. Install Qt6 via the [Qt Online Installer](https://www.qt.io/download-qt-installer)
3. Install OpenCV via [vcpkg](https://github.com/microsoft/vcpkg): `vcpkg install opencv4`
4. Download ONNX Runtime from [GitHub releases](https://github.com/microsoft/onnxruntime/releases)
5. Point Meson at the dependencies with a [native file](https://mesonbuild.com/Native-environment.html) and/or `-Donnxruntime_root=...`

## Build

```bash
meson setup build
meson compile -C build
```

On Ubuntu/Debian, Qt6 ships no pkg-config files and the generic `qmake` resolves
to Qt5. A native file pointing at the Qt6 tools is included — pass it at setup:

```bash
meson setup build --native-file meson/ubuntu-qt6.ini
```

### Custom dependency paths

```bash
# OpenCV: prepend its .pc dir to PKG_CONFIG_PATH.
# Qt6: use a native file's [binaries] section (see meson/ubuntu-qt6.ini).
# ONNX Runtime: auto-discovered under /opt/onnxruntime* or /usr/local/onnxruntime*,
# otherwise pass an explicit path:
meson setup build -Donnxruntime_root=/path/to/onnxruntime
```

### Build options (`-D<name>=<value>`)

| Option | Default | Description |
|--------|---------|-------------|
| `tests` | `true` | Build unit tests |
| `blackmagic` | `false` | Enable Blackmagic DeckLink support |
| `directml` | `false` | Enable the ONNX Runtime DirectML execution provider (Windows GPU) |
| `onnxruntime_root` | `''` | Path to a custom ONNX Runtime install |

## Run

```bash
# Uses config.yaml from current directory
./build/MoCapStudio

# Custom config path
./build/MoCapStudio /path/to/config.yaml
```

## Tests

```bash
meson test -C build
```

## Configuration

Edit `config.yaml` to configure cameras, pose estimation, triangulation, and skeleton settings. Example:

```yaml
capture:
  target_fps: 60
  sync_mode: "software"
  max_sync_skew_ms: 5

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
  backend: "onnxruntime"
  model: "resources/models/rtmpose-m-wholebody.onnx"
  device: "cuda:0"
  detection_threshold: 0.5
  keypoint_threshold: 0.3

triangulation:
  min_views: 2
  ransac_enabled: true
  ransac_threshold_px: 5.0
  temporal_filter: "butterworth"
  filter_cutoff_hz: 6.0

skeleton:
  definition: "body_25"
  ik_solver: "analytical"
  joint_limits_enabled: true

gui:
  canvas_fps: 60
  default_render_layers: ["grid", "markers", "skeleton"]
  colour_palette: "oklab_12"
```

## Calibrating two cameras for 3D

1. Keep both cameras fixed, with overlapping views of the capture area. Create or
   open a session, then choose **Cameras → Calibrate** and select both cameras.
2. Use a flat, rigid checkerboard. Enter its **inner corner** counts and measured
   square width in metres (the default is 9 × 6 inner corners, 25 mm squares).
3. Hold the entire board still and visible in both cameras before pressing
   **Capture Frame**. Move and tilt it between captures, covering different parts
   of the shared field of view and different distances. Do not flip the board.
   Collect at least eight paired views; partial detections are not saved.
4. Run calibration and review the lens/stereo RMS errors and camera separation.
   Errors above 2 pixels prevent completion: recapture sharper, varied views.
   **Finish** writes intrinsics and extrinsics into the session and updates the
   camera configuration. Selecting only one camera performs lens calibration and
   clears that camera's old extrinsics; it cannot enable 3D by itself.
5. Start capture with **F5**, with the same person visible in both cameras.

The first selected camera defines the world origin, with X right, Y up and Z
behind that camera; distances are in metres. This does not estimate the floor.
Moving a camera, changing its zoom/focus or changing the stream resolution requires
recalibration. A newly calibrated rig replaces the world frame, so unselected
cameras' old extrinsics are removed from the active configuration.

The wizard accepts live frame pairs up to 100 ms apart, and requires the board to
remain stationary during each capture. This is separate from the stricter
`capture.max_sync_skew_ms` used for motion capture. Camera timestamps measure host
frame arrival, not hardware exposure synchronization; differing RTSP latency can
still reduce accuracy for moving subjects. Cross-camera person matching uses
calibrated reprojection agreement and works best with one person in the shared view.

The stereo solve uses fixed lens parameters with OpenCV's
[`stereoCalibrate`](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html).

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
subprojects/        # Meson wrap files for auto-fetched dependencies
meson/              # Native files (e.g. Qt6 tool paths on Ubuntu/Debian)
```

## License

MIT — see [LICENSE](LICENSE).
