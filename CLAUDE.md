# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MoCap Studio is a desktop application for real-time, markerless human motion capture. It ingests synchronised video from multiple cameras, runs 2D pose estimation, triangulates into 3D markers, and fits a skeletal model to produce joint rotations. The full technical specification is in `docs/MoCapStudio_Spec.md`.

**Status:** Pre-implementation. The `src/` directory is empty. Only the spec and project scaffolding exist.

## Build Commands

C++17 project using CMake >= 3.22 with Qt6 integration:

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DQt6_DIR=/path/to/qt6
cmake --build . --parallel
```

No test framework has been configured yet.

## Architecture

Five-stage pipeline with a Qt GUI shell:

```
Capture Engine → 2D Pose Estimator → Triangulation → Skeleton Solver → Storage/Export
```

**Three-layer data model:**
- **L1 (Raw2DPose):** Per-camera pixel keypoints, bboxes, confidences
- **L2 (Pose3D):** World-space 3D marker positions after multi-view triangulation
- **L3 (SkeletonPose):** Joint rotations (quaternion + Euler), root transform after IK fitting

Each layer is independently queryable and exportable.

**Key modules:**

| Module | Responsibility |
|---|---|
| Capture Engine | Camera discovery, multi-cam sync, frame acquisition via `ICameraSource` interface |
| 2D Pose Estimator | Per-frame keypoint detection via `IPoseEstimator` interface (default: RTMPose-L ONNX) |
| Triangulation | Multi-view 2D→3D lifting (DLT), RANSAC outlier rejection, Butterworth temporal filtering |
| Skeleton Solver | IK fitting of parametric skeleton (BODY_25 default) to 3D marker cloud |
| Storage/Export | Session directory persistence; exports to CSV, JSON, C3D, BVH, FBX, USD |
| Qt GUI | Camera feeds, 3D canvas (`MoCapCanvas` QOpenGLWidget), timeline, inspector |

**Threading:** Pipeline stages run on worker threads. GUI updates via Qt signals/slots with `Qt::QueuedConnection`. Heavy data passed via `QSharedPointer`.

## Key Dependencies

| Library | Version | Purpose |
|---|---|---|
| Qt | 6.5+ | GUI (Widgets + OpenGL) |
| OpenCV | 4.8+ | Camera I/O, calibration |
| ONNX Runtime | 1.16+ | Pose model inference |
| Eigen | 3.4+ | Linear algebra |
| spdlog | 1.12+ | Logging |
| yaml-cpp | 0.7+ | Config parsing (`config.yaml`) |
| nlohmann/json | 3.11+ | JSON serialisation |
| ezc3d | latest | C3D file I/O |

## Platform Targets

- **Primary:** Linux (Ubuntu 22.04+), GCC 12+ / Clang 15+
- **Supported:** Windows 10/11, MSVC 2022+
- **Planned:** macOS 13+

## Configuration

Application config is `config.yaml` (YAML format via yaml-cpp). Camera intrinsics stored as per-camera YAML files. Extrinsics as JSON. Session data stored in timestamped directories with binary layer files.
