# MoCap Studio — Full Implementation Design

> **Date:** 2026-03-03
> **Scope:** Full spec implementation (all pipeline stages, full GUI, all export formats)
> **Approach:** Bottom-up (build system → core types → pipeline stages → GUI)

---

## Project Structure

```
MoCap-Studio/
├── CMakeLists.txt              # Root CMake
├── cmake/                      # FindONNXRuntime.cmake, etc.
├── config.yaml                 # Default app config
├── src/
│   ├── main.cpp
│   ├── core/                   # Data types, interfaces, config
│   ├── capture/                # Camera backends + frame broker
│   ├── pose/                   # 2D pose estimation + person tracking
│   ├── triangulation/          # Multi-view 3D lifting + filtering
│   ├── skeleton/               # IK solver
│   ├── storage/                # Session management + exporters
│   └── gui/                    # Qt widgets + 3D canvas
├── tests/                      # Google Test suite
├── resources/                  # Skeletons, shaders, icons
└── docs/
```

## Build System

- CMake >= 3.22, C++17
- FetchContent: Eigen, spdlog, nlohmann/json, Google Test, ezc3d
- find_package: Qt6 (Widgets, OpenGL, OpenGLWidgets), OpenCV, ONNX Runtime
- Optional: Assimp (FBX export)

## Core Data Types (core/types.h)

All types from spec sections 4.1–4.4:
- Keypoint2D, Raw2DPose (L1)
- Marker3D, Pose3D (L2)
- JointRotation, SkeletonPose (L3)
- CapturedFrame, FrameSet, FramePacket (pipeline)

Vec3f = Eigen::Vector3f, Quaternion = Eigen::Quaternionf.

## Interfaces

- ICameraSource: open, close, isOpened, grabFrame, intrinsics, id, displayName
- IPoseEstimator: initialize, estimate(cv::Mat) → vector<Raw2DPose>

## Pipeline Stages

### Capture Engine
- 4 backends: UsbCameraSource, IpCameraSource, VideoFileCameraSource, BlackmagicCameraSource
- FrameBroker: thread-per-camera, timestamp-based FrameSet assembly, configurable max skew

### 2D Pose Estimation
- OnnxPoseEstimator: RTMPose ONNX model, preprocessing (resize/normalize/pad), heatmap/simcc decoding
- PersonTracker: Hungarian algorithm, cross-camera matching via epipolar + IoU + appearance

### Triangulation
- DLT triangulation for keypoints visible in >= 2 views
- RANSAC outlier rejection for >= 3 views
- TemporalFilter: Butterworth low-pass, configurable cutoff (default 6 Hz)

### Skeleton Solver
- SkeletonDefinition from JSON (BODY_25 default)
- Analytical IK for simple chains, optimization-based fallback
- Joint-limit constraints, quaternion output

### Storage/Export
- SessionManager: timestamped directories, metadata JSON
- BinaryIO: FramePacket format for L1/L2/L3
- Exporters: CSV, JSON, C3D (ezc3d), BVH, FBX (Assimp), USD

## Qt GUI

- MainWindow: QMainWindow with dock-based layout
- CameraFeedWidget: live camera thumbnails with 2D keypoint overlay
- MoCapCanvas: QOpenGLWidget, orbit/pan/zoom, ground grid, markers, skeleton, trails, frustums
- InspectorPanel: context-sensitive property display
- TimelineWidget: scrub bar + transport controls
- Dialogs: SessionDialog, CalibrationWizard, ExportDialog
- Per-person color coding via Oklab hue rotation
- Cross-thread data flow via Qt::QueuedConnection signals, QSharedPointer for heavy data

## Testing

Google Test via FetchContent. Tests for:
- Core types serialization
- Triangulation math
- Skeleton solver
- Temporal filter
- Binary I/O
