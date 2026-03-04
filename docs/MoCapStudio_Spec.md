# MoCap Studio — Technical Specification

> **Version:** 0.1.0-draft
> **Purpose:** Markerless multi-person motion capture with multi-camera fusion, Qt-based GUI, and structured motion data output.

---

## 1. System Overview

MoCap Studio is a desktop application for real-time, markerless human motion capture. It ingests synchronised video from one or more cameras, runs 2D pose estimation on each view, triangulates the results into 3D marker trajectories, and fits a skeletal model to produce joint rotations. A Qt-based GUI provides live visualisation and session management.

### 1.1 Design Principles

- **Markerless capture** — no physical markers or suits; all tracking is vision-based.
- **Multi-person** — track N subjects simultaneously with persistent identity across frames.
- **Multi-camera** — fuse 2 … M calibrated camera views to resolve depth and occlusion.
- **Layered data pipeline** — Raw 2D → Processed 3D → Skeletal rotations, each layer accessible independently.
- **Offline-first, live-preview** — optimised for post-capture batch processing with optional real-time preview at reduced fidelity.

---

## 2. Architecture

```
┌────────────────────────────────────────────────────────┐
│                      Qt GUI Shell                      │
│  ┌──────────┐  ┌────────────┐  ┌────────────────────┐ │
│  │ Camera   │  │ 3D Canvas  │  │ Timeline / Data    │ │
│  │ Panels   │  │ (Body Viz) │  │ Inspector          │ │
│  └────┬─────┘  └─────▲──────┘  └──────▲─────────────┘ │
│       │               │               │               │
├───────┼───────────────┼───────────────┼───────────────┤
│       │          Core Pipeline        │               │
│       ▼                               │               │
│  ┌─────────┐   ┌───────────┐   ┌─────┴─────┐         │
│  │ Capture  │──▶│  2D Pose  │──▶│ Triangu-  │         │
│  │ Engine   │   │ Estimator │   │ lation    │         │
│  └─────────┘   └───────────┘   └─────┬─────┘         │
│                                      │               │
│                                ┌─────▼─────┐         │
│                                │ Skeleton   │         │
│                                │ Solver     │         │
│                                └─────┬─────┘         │
│                                      │               │
│                                ┌─────▼─────┐         │
│                                │  Storage   │         │
│                                │  / Export  │         │
│                                └───────────┘         │
└────────────────────────────────────────────────────────┘
```

### 2.1 Module Inventory

| Module | Responsibility | Key Dependencies |
|---|---|---|
| **Capture Engine** | Camera discovery, synchronisation, frame acquisition | OpenCV `VideoCapture`, V4L2/DirectShow/AVFoundation |
| **2D Pose Estimator** | Per-frame, per-camera keypoint detection (bottom-up or top-down) | ONNX Runtime or LibTorch; model weights (e.g. RTMPose, ViTPose, HRNet) |
| **Triangulation** | Multi-view 2D→3D lifting, camera calibration, correspondence | OpenCV `triangulatePoints`, custom RANSAC |
| **Skeleton Solver** | Inverse-kinematics fitting, joint rotation extraction | Custom IK or third-party (e.g. Pinocchio, Biomechanical solver) |
| **Storage / Export** | Session persistence, file I/O | C3D, BVH, FBX, CSV, JSON |
| **Qt GUI** | Windowing, widgets, 3D canvas, user interaction | Qt 6 (Widgets + OpenGL/Vulkan) |

---

## 3. Camera Subsystem

### 3.1 Camera Source Abstraction

```cpp
// ICameraSource — unified interface for all camera backends
class ICameraSource {
public:
    virtual ~ICameraSource() = default;

    virtual bool open(const CameraConfig& config) = 0;
    virtual void close() = 0;
    virtual bool isOpened() const = 0;

    // Grab a synchronised frame; blocks until available or timeout.
    virtual bool grabFrame(CapturedFrame& out, int timeout_ms = 100) = 0;

    virtual CameraIntrinsics intrinsics() const = 0;
    virtual std::string       id() const = 0;        // unique device id
    virtual std::string       displayName() const = 0;
};
```

Supported backends (implement `ICameraSource`):

| Backend | Platform | Notes |
|---|---|---|
| `UsbCameraSource` | Cross-platform | OpenCV `VideoCapture(index)` or V4L2 on Linux |
| `IpCameraSource` | Cross-platform | RTSP / HTTP MJPEG streams |
| `VideoFileCameraSource` | Cross-platform | Replay from recorded video files |
| `BlackmagicCameraSource` | Win/macOS/Linux | DeckLink SDK (optional plugin) |

### 3.2 Multi-Camera Synchronisation

- **Hardware sync (preferred):** genlock / trigger cable — cameras fire on shared signal.
- **Software sync (fallback):** timestamp-based nearest-neighbour matching with configurable max-skew tolerance (default: 5 ms).
- **Frame broker** collects frames from all sources, groups them into `FrameSet` bundles aligned by timestamp, and publishes to the pipeline.

### 3.3 Camera Calibration

| Data | Storage |
|---|---|
| Intrinsics (fx, fy, cx, cy, distortion) | Per-camera YAML/JSON |
| Extrinsics (R, t relative to world origin) | Per-session calibration file |

Calibration workflow:

1. **Intrinsic calibration** — checkerboard / ChArUco board captured per camera (`cv::calibrateCamera`).
2. **Extrinsic calibration** — wand-based or checkerboard-based multi-view calibration (`cv::stereoCalibrate` extended to N views), or bundle adjustment.
3. **Refinement** — optional bundle-adjustment pass over detected keypoints to minimise reprojection error.

---

## 4. Pose Estimation Pipeline

### 4.1 Stage 1 — Raw 2D Keypoint Detection

- Run a pose estimator on every camera frame independently.
- **Model-agnostic** — support pluggable backends via `IPoseEstimator` interface.
- Default: RTMPose-L (ONNX) for speed/accuracy balance.
- Output per frame per camera per detected person:

```
Raw2DPose {
    person_id   : int            // detector-local person index (not yet globally tracked)
    keypoints   : Vec<Keypoint2D>
    bbox        : Rect           // detection bounding box
    confidence  : float          // detection-level confidence
}

Keypoint2D {
    name  : string   // e.g. "left_shoulder"
    index : int      // index in the skeleton topology
    x     : float    // pixel coordinate (undistorted)
    y     : float    // pixel coordinate (undistorted)
    conf  : float    // keypoint confidence [0, 1]
}
```

- **Multi-person:** bottom-up models (e.g. HigherHRNet) detect all persons in one pass; top-down models require a preceding detector (e.g. YOLO / RTMDet) then crop-and-estimate.

### 4.2 Stage 2 — Multi-View Correspondence & Identity Tracking

- **Cross-camera matching:** associate 2D detections of the same physical person across cameras using epipolar-geometry consistency and appearance embedding similarity.
- **Temporal tracking:** maintain persistent `GlobalPersonID` across frames (Hungarian algorithm or similar assignment on per-frame cost matrix combining IoU, appearance, and epipolar distance).
- Output: `TrackedPerson2D` — same as `Raw2DPose` but with a stable `global_person_id`.

### 4.3 Stage 3 — 3D Triangulation

For each tracked person and each keypoint visible in ≥ 2 cameras:

1. Undistort 2D coordinates using camera intrinsics.
2. Compute 3D position via DLT (Direct Linear Transform) or mid-point triangulation.
3. Apply RANSAC if ≥ 3 views to reject outlier camera views.
4. Filter trajectories temporally (Butterworth low-pass or Savitzky-Golay).

```
Marker3D {
    name       : string     // keypoint name
    index      : int
    position   : Vec3f      // (x, y, z) in world coordinates, metres
    confidence : float      // aggregated multi-view confidence
    n_views    : int        // number of cameras that contributed
}

Pose3D {
    global_person_id : int
    timestamp        : double       // seconds from session start
    markers          : Vec<Marker3D>
}
```

### 4.4 Stage 4 — Skeletal Fitting & Joint Rotations

- Fit a parametric skeleton (defined by a `SkeletonDefinition`) to the 3D marker cloud.
- Solve for joint angles using analytical IK or optimisation-based IK (minimise marker residual + joint-limit penalties).
- Output per person per frame:

```
SkeletonPose {
    global_person_id : int
    timestamp        : double
    root_position    : Vec3f               // pelvis / hips world position
    root_rotation    : Quaternion           // pelvis / hips world orientation
    joint_rotations  : Vec<JointRotation>
}

JointRotation {
    joint_name  : string       // e.g. "left_elbow"
    joint_index : int
    rotation    : Quaternion   // local rotation relative to parent joint
    euler_xyz   : Vec3f        // convenience Euler angles (degrees)
}
```

### 4.5 Skeleton Definition

```
SkeletonDefinition {
    name       : string                // e.g. "MoCap_Body25", "SMPL", "Custom"
    joints     : Vec<JointDef>
    keypoint_to_joint_map : Map<int, int>  // maps 2D keypoint index → joint index
}

JointDef {
    index       : int
    name        : string
    parent      : int          // -1 for root
    rest_offset : Vec3f        // offset from parent in rest pose (T-pose), metres
    twist_axis  : Vec3f        // primary rotation axis hint
    limits      : JointLimits  // min/max per DoF (optional)
}
```

A default `BODY_25` skeleton definition shall ship with the application; users can load custom definitions from JSON.

---

## 5. Data Types & Serialisation

### 5.1 Three-Layer Data Model

| Layer | Type | Content | When Available |
|---|---|---|---|
| **L1 — Raw 2D** | `Raw2DPose` | Per-camera pixel keypoints, bboxes, confidences | After Stage 1 |
| **L2 — Processed 3D** | `Pose3D` | World-space 3D marker positions, filtered trajectories | After Stage 3 |
| **L3 — Skeletal** | `SkeletonPose` | Joint rotations (quaternion + Euler), root transform | After Stage 4 |

All three layers are stored per-frame per-person and are independently queryable/exportable.

### 5.2 Session Container

A MoCap session is persisted as a directory:

```
session_2025-06-15_14-30-00/
├── meta.json                  # session metadata (fps, duration, camera list, skeleton def)
├── calibration/
│   ├── intrinsics/            # per-camera YAML
│   └── extrinsics.json        # relative transforms
├── raw_video/                 # optional recorded source video
│   ├── cam_0.mp4
│   └── cam_1.mp4
├── data/
│   ├── raw_2d.bin             # L1 binary (or .csv / .json)
│   ├── markers_3d.bin         # L2 binary
│   └── skeleton.bin           # L3 binary
└── exports/                   # user-generated exports
    ├── skeleton.bvh
    ├── skeleton.fbx
    └── markers.c3d
```

### 5.3 Export Formats

| Format | Layers | Notes |
|---|---|---|
| **CSV / JSON** | L1, L2, L3 | Human-readable, per-layer export |
| **C3D** | L2 | Industry-standard marker format |
| **BVH** | L3 | Skeletal animation (Biovision) |
| **FBX** | L3 | 3D authoring tools (Blender, Maya, Unreal) |
| **USD** | L3 | NVIDIA Omniverse / virtual production pipelines |

### 5.4 Binary Wire Format (Internal)

For real-time streaming between pipeline stages and to the GUI:

```
FramePacket {
    magic       : u32     = 0x4D435030   // "MCP0"
    version     : u8
    layer       : u8      // 1=Raw2D, 2=Pose3D, 3=Skeleton
    person_id   : u16
    timestamp   : f64     // seconds
    n_items     : u16     // number of keypoints/markers/joints
    payload     : [u8]    // layer-specific packed structs
}
```

---

## 6. Qt GUI

### 6.1 Technology Stack

| Component | Choice |
|---|---|
| Framework | Qt 6 (Widgets) |
| 3D Rendering | QOpenGLWidget (OpenGL 4.1+ core profile) or Qt 3D |
| Language | C++ 17 (GUI + core pipeline) with Python bindings (optional) |
| Build system | CMake ≥ 3.22 with Qt6 integration |

### 6.2 Window Layout

```
┌──────────────────────────────────────────────────────────────┐
│ Menu Bar  [File] [Session] [Cameras] [View] [Export] [Help]  │
├──────────┬───────────────────────────────┬───────────────────┤
│          │                               │                   │
│  Camera  │                               │  Properties /     │
│  Feed    │     3D Canvas                 │  Inspector        │
│  Panel   │     (Body Visualisation)      │                   │
│          │                               │  - Selected person│
│  ┌─────┐ │     ● Skeleton overlay        │  - Joint angles   │
│  │Cam 0│ │     ● Marker point cloud      │  - Marker stats   │
│  └─────┘ │     ● Ground grid             │  - Confidence map │
│  ┌─────┐ │     ● Orbit / pan / zoom      │                   │
│  │Cam 1│ │                               │                   │
│  └─────┘ │                               │                   │
│  ┌─────┐ │                               │                   │
│  │Cam 2│ │                               │                   │
│  └─────┘ │                               │                   │
├──────────┴───────────────────────────────┴───────────────────┤
│  Timeline ◄━━━━━━━━━━━━━━●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━► │
│  [▶ Play] [⏸ Pause] [⏹ Stop] [⏺ Record]    00:01:32 / 00:05:00  │
├──────────────────────────────────────────────────────────────┤
│  Status: ● 3 cameras connected  │  ● 2 persons tracked  │ 60fps │
└──────────────────────────────────────────────────────────────┘
```

### 6.3 3D Canvas (Body Visualisation)

The central canvas is a `QOpenGLWidget` subclass (`MoCapCanvas`) responsible for rendering tracked body data in real time.

**Render layers (toggleable):**

| Layer | Description | Default |
|---|---|---|
| Ground grid | Reference plane with axis indicators | ON |
| 3D Markers | Spheres at each `Marker3D` position, colour-coded by confidence | ON |
| Skeleton | Bones drawn between parent→child joints; joint spheres at rotation centres | ON |
| Marker trails | Fading trajectory tails (configurable length) | OFF |
| Camera frustums | Wireframe frustums showing each camera's FOV and position | OFF |
| Bounding volumes | Per-person bounding box | OFF |

**Interaction:**

| Input | Action |
|---|---|
| Left-drag | Orbit camera |
| Right-drag | Pan camera |
| Scroll | Zoom |
| Middle-click marker/joint | Select → show details in Inspector |
| `F` key | Frame selection (zoom-to-fit selected person) |
| `1` / `2` / `3` | Toggle render layer visibility |

**Colour coding (per person):**

Assign each `GlobalPersonID` a distinct colour from a perceptually uniform palette (e.g. Oklab hue rotation). All markers and bones for a person share that colour. Confidence is mapped to opacity (low confidence → semi-transparent).

### 6.4 Key Widgets

| Widget | Class | Purpose |
|---|---|---|
| Camera Feed Panel | `CameraFeedWidget` | Thumbnail grid of live camera views; click to enlarge; optional 2D keypoint overlay |
| 3D Canvas | `MoCapCanvas` | Primary body data visualisation (see §6.3) |
| Timeline | `TimelineWidget` | Scrub bar, transport controls, frame-accurate seeking during playback/review |
| Inspector | `InspectorPanel` | Context-sensitive property display for selected person/joint/marker |
| Session Manager | `SessionDialog` | New / open / recent sessions, recording settings |
| Calibration Wizard | `CalibrationWizard` | Step-by-step camera calibration flow |
| Export Dialog | `ExportDialog` | Choose layers, format, frame range, coordinate system |

### 6.5 Signals & Slots (Data Flow to GUI)

```
Pipeline Thread                          GUI Thread
─────────────────                        ──────────────────
FrameBroker ──signal(FrameSet)──────────▶ CameraFeedWidget::onFrameSet
PoseEstimator ──signal(Raw2DPoses)──────▶ CameraFeedWidget::onPoses2D (overlay)
Triangulator ──signal(Vec<Pose3D>)──────▶ MoCapCanvas::onPose3DUpdate
SkeletonSolver ──signal(Vec<SkeletonPose>)▶ MoCapCanvas::onSkeletonUpdate
                                           InspectorPanel::onSkeletonUpdate
```

All cross-thread signals use `Qt::QueuedConnection`. Heavy data (images, point clouds) is passed via `QSharedPointer` to avoid copies.

---

## 7. Configuration

### 7.1 Application Config (`config.yaml`)

```yaml
capture:
  target_fps: 60
  sync_mode: "software"          # "hardware" | "software"
  max_sync_skew_ms: 5

cameras:
  - id: "cam_0"
    type: "usb"                  # "usb" | "ip" | "file"
    device_index: 0
    resolution: [1920, 1080]
    intrinsics_file: "calibration/cam_0_intrinsics.yaml"
  - id: "cam_1"
    type: "ip"
    url: "rtsp://192.168.1.100:554/stream"
    resolution: [1920, 1080]
    intrinsics_file: "calibration/cam_1_intrinsics.yaml"

pose_estimation:
  backend: "onnxruntime"         # "onnxruntime" | "libtorch"
  model: "rtmpose_l_body25.onnx"
  device: "cuda:0"               # "cpu" | "cuda:N"
  detection_threshold: 0.5
  keypoint_threshold: 0.3

triangulation:
  min_views: 2
  ransac_enabled: true
  ransac_threshold_px: 5.0
  temporal_filter: "butterworth"
  filter_cutoff_hz: 6.0

skeleton:
  definition: "body_25"          # or path to custom JSON
  ik_solver: "analytical"        # "analytical" | "optimisation"
  joint_limits_enabled: true

gui:
  canvas_fps: 60
  default_render_layers: ["grid", "markers", "skeleton"]
  colour_palette: "oklab_12"
```

---

## 8. Performance Targets

| Metric | Target | Notes |
|---|---|---|
| Input frame rate | 30–120 fps per camera | Depends on camera hardware |
| 2D pose latency | < 30 ms per frame (GPU) | Batch across cameras where possible |
| Triangulation latency | < 2 ms per frame | CPU, per-person |
| Skeleton solve latency | < 5 ms per frame | CPU, per-person |
| End-to-end latency (preview) | < 80 ms | From capture to canvas render |
| Max simultaneous persons | 10 | Soft limit; configurable |
| Max simultaneous cameras | 8 | Hardware-dependent |
| GPU VRAM | < 4 GB | Pose model + render context |

---

## 9. Dependencies

| Library | Version | Purpose |
|---|---|---|
| Qt | 6.5+ | GUI framework |
| OpenCV | 4.8+ | Camera I/O, calibration, image processing |
| ONNX Runtime | 1.16+ | Pose model inference |
| Eigen | 3.4+ | Linear algebra (triangulation, IK) |
| spdlog | 1.12+ | Logging |
| yaml-cpp | 0.7+ | Configuration parsing |
| nlohmann/json | 3.11+ | JSON serialisation |
| ezc3d | (latest) | C3D file I/O |
| Assimp or FBX SDK | — | FBX export |

---

## 10. Build & Platform Support

| Platform | Compiler | Status |
|---|---|---|
| Linux (Ubuntu 22.04+) | GCC 12+ / Clang 15+ | Primary |
| Windows 10/11 | MSVC 2022+ | Supported |
| macOS 13+ | Apple Clang 15+ | Planned |

Build:

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DQt6_DIR=/path/to/qt6
cmake --build . --parallel
```

---

## 11. Future Extensions (Out of Scope for v0.1)

- Hand and finger tracking (21 keypoints per hand).
- Facial landmark capture and blendshape output.
- Real-time streaming output (LiveLink, OSC, VRPN).
- GPU-accelerated triangulation (CUDA kernels).
- Deep-learning-based 3D lifting (single-camera mode with learned depth prior).
- Cloud-based batch processing.
- USD Live / Omniverse Connector for virtual production.
