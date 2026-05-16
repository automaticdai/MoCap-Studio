# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MoCap Studio is a desktop application for real-time, markerless human motion capture. It ingests synchronised video from multiple cameras, runs 2D pose estimation, triangulates into 3D markers, and fits a skeletal model to produce joint rotations. The full technical specification is in `docs/MoCapStudio_Spec.md`; the bottom-up implementation plan is in `docs/plans/2026-03-03-full-implementation-design.md`.

**Status:** All five pipeline stages, the Qt GUI shell, all export formats, and the gtest suite are implemented and building. Known stubs: Blackmagic DeckLink backend (`src/capture/blackmagic_camera_source.cpp` logs a warning — no SDK integration), and the USD exporter writes plain USDA text rather than going through the USD SDK (`src/storage/exporters/usd_exporter.cpp`).

## Build Commands

C++17 project using CMake >= 3.22 with Qt6 integration:

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel
```

Custom dependency paths (when Qt6/OpenCV/ONNX Runtime aren't on standard paths):

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DQt6_DIR=/path/to/qt6/lib/cmake/Qt6 \
  -DOpenCV_DIR=/path/to/opencv/lib/cmake/opencv4 \
  -DONNXRUNTIME_ROOT=/path/to/onnxruntime
```

CMake options:

| Option | Default | Description |
|---|---|---|
| `MOCAP_BUILD_TESTS` | `ON` | Build the Google Test suite under `tests/` |
| `MOCAP_ENABLE_BLACKMAGIC` | `OFF` | Compile the Blackmagic DeckLink backend (currently a stub) |
| `MOCAP_ENABLE_DIRECTML` | `OFF` | Enable the ONNX Runtime DirectML execution provider (Windows GPU). Requires an ORT build with DML and `dml_provider_factory.h` on the include path |

Run tests:

```bash
cd build && ctest --output-on-failure
```

## Architecture

Five-stage pipeline with a Qt GUI shell:

```
Capture Engine → 2D Pose Estimator → Triangulation → Skeleton Solver → Storage/Export
```

**Three-layer data model** (defined in `src/core/types.h`):
- **L1 (Raw2DPose):** Per-camera pixel keypoints, bboxes, confidences
- **L2 (Pose3D):** World-space 3D marker positions after multi-view triangulation
- **L3 (SkeletonPose):** Joint rotations (quaternion + Euler), root transform after IK fitting

Each layer is independently queryable and exportable.

**Module map (source layout):**

| Module | Directory | Notes |
|---|---|---|
| Core | `src/core/` | `types.h`, `config.{h,cpp}`, `skeleton_definition.{h,cpp}`, `camera_intrinsics.{h,cpp}` |
| Capture | `src/capture/` | `ICameraSource` interface, USB/IP/video-file/Blackmagic backends, `FrameBroker` for multi-cam sync |
| 2D Pose | `src/pose/` | `IPoseEstimator` interface, `OnnxPoseEstimator` (RTMPose), `PersonTracker` (Hungarian) |
| Triangulation | `src/triangulation/` | DLT lifting + RANSAC (`Triangulator`), Butterworth low-pass (`TemporalFilter`) |
| Skeleton | `src/skeleton/` | `SkeletonSolver` — analytical IK with optimisation fallback |
| Storage | `src/storage/` | `SessionManager`, `BinaryIO`, exporters under `exporters/` (CSV, JSON, C3D, BVH, FBX, USD) |
| GUI | `src/gui/` | `MainWindow`, `MoCapCanvas` (QOpenGLWidget), `CameraFeedWidget`, `TimelineWidget`, `InspectorPanel`, dialogs (session/calibration/export/camera-management/settings) |

**Threading:** Pipeline stages run on worker threads. GUI updates via Qt signals/slots with `Qt::QueuedConnection`. Heavy data passed via `QSharedPointer`.

## Key Dependencies

Required (must be installed on the system):

| Library | Version | Purpose |
|---|---|---|
| Qt | 6.5+ | GUI (Widgets + OpenGL + OpenGLWidgets) |
| OpenCV | 4.0+ | Camera I/O, calibration |
| ONNX Runtime | 1.16+ | Pose model inference |

Auto-fetched via CMake `FetchContent` if not found on the system:

| Library | Version | Purpose |
|---|---|---|
| Eigen | 3.4 | Linear algebra |
| spdlog | 1.14.1 | Logging (always fetched to avoid system `fmt` clashes) |
| yaml-cpp | 0.8 | Config parsing (`config.yaml`) |
| nlohmann/json | 3.11.3 | JSON serialisation |
| ezc3d | 1.5.8 | C3D file I/O |
| Google Test | 1.14 | Unit tests |

Optional: Assimp (FBX export — feature compiled in when found).

## Platform Targets

- **Primary:** Linux (Ubuntu 22.04+), GCC 12+ / Clang 15+
- **Supported:** Windows 10/11, MSVC 2022+
- **Planned:** macOS 13+

## Configuration

Application config is `config.yaml` at the repo root (YAML via yaml-cpp); the binary takes an optional config path as its first arg. Camera intrinsics are stored as per-camera YAML, extrinsics as JSON. Session data lives in timestamped directories with binary L1/L2/L3 layer files written by `BinaryIO`. Skeleton definitions live in `resources/skeletons/` (BODY_25 default); GL shaders in `resources/shaders/`.
