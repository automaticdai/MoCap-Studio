#include "gui/main_window.h"
#include "gui/camera_feed_widget.h"
#include "gui/mocap_canvas.h"
#include "gui/timeline_widget.h"
#include "gui/inspector_panel.h"
#include "gui/session_dialog.h"
#include "gui/calibration_wizard.h"
#include "gui/export_dialog.h"

#include "capture/usb_camera_source.h"
#include "capture/ip_camera_source.h"
#include "capture/video_file_camera_source.h"
#include "storage/exporters/csv_exporter.h"
#include "storage/exporters/json_exporter.h"
#include "storage/exporters/c3d_exporter.h"
#include "storage/exporters/bvh_exporter.h"
#include "storage/exporters/fbx_exporter.h"
#include "storage/exporters/usd_exporter.h"

#include <QMenuBar>
#include <QStatusBar>
#include <QDockWidget>
#include <QMessageBox>
#include <QFileDialog>
#include <spdlog/spdlog.h>
#include <filesystem>

namespace mocap {

MainWindow::MainWindow(const std::string& config_path, QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("MoCap Studio");
    resize(1600, 900);

    // Initialize pipeline components
    frame_broker_ = std::make_shared<FrameBroker>(this);
    pose_estimator_ = std::make_unique<OnnxPoseEstimator>();
    person_tracker_ = std::make_unique<PersonTracker>();
    triangulator_ = std::make_unique<Triangulator>();
    temporal_filter_ = std::make_unique<TemporalFilter>(
        TemporalFilter::Type::Butterworth, 6.0, 60.0);
    skeleton_solver_ = std::make_unique<SkeletonSolver>();
    session_manager_ = std::make_unique<SessionManager>();

    // Set default skeleton
    skeleton_solver_->setSkeletonDefinition(SkeletonDefinition::defaultBody25());

    setupMenuBar();
    setupStatusBar();
    setupDockWidgets();
    setupConnections();
    loadConfig(config_path);
}

MainWindow::~MainWindow() {
    if (capturing_) {
        onStopCapture();
    }
}

void MainWindow::setupMenuBar() {
    auto* fileMenu = menuBar()->addMenu("&File");
    fileMenu->addAction("&New Session...", this, &MainWindow::onNewSession, QKeySequence::New);
    fileMenu->addAction("&Open Session...", this, &MainWindow::onOpenSession, QKeySequence::Open);
    fileMenu->addSeparator();
    fileMenu->addAction("E&xit", this, &QWidget::close, QKeySequence::Quit);

    auto* sessionMenu = menuBar()->addMenu("&Session");
    sessionMenu->addAction("&Start Capture", this, &MainWindow::onStartCapture, Qt::Key_F5);
    sessionMenu->addAction("S&top Capture", this, &MainWindow::onStopCapture, Qt::Key_F6);

    auto* camerasMenu = menuBar()->addMenu("&Cameras");
    camerasMenu->addAction("Camera &Setup...", this, &MainWindow::onCameraSetup);
    camerasMenu->addAction("&Calibrate...", this, &MainWindow::onCalibrate);

    auto* viewMenu = menuBar()->addMenu("&View");
    viewMenu->setObjectName("viewMenu");

    auto* exportMenu = menuBar()->addMenu("&Export");
    exportMenu->addAction("&Export Data...", this, &MainWindow::onExport);

    auto* helpMenu = menuBar()->addMenu("&Help");
    helpMenu->addAction("&About", this, &MainWindow::onAbout);
}

void MainWindow::setupStatusBar() {
    status_cameras_ = new QLabel("Cameras: 0");
    status_persons_ = new QLabel("Persons: 0");
    status_fps_ = new QLabel("FPS: 0");

    statusBar()->addPermanentWidget(status_cameras_);
    statusBar()->addPermanentWidget(status_persons_);
    statusBar()->addPermanentWidget(status_fps_);

    statusBar()->showMessage("Ready");

    fps_timer_ = new QTimer(this);
    connect(fps_timer_, &QTimer::timeout, this, &MainWindow::updateStatusBar);
    fps_timer_->start(1000);
}

void MainWindow::setupDockWidgets() {
    // Camera Feed (left dock)
    camera_feed_widget_ = new CameraFeedWidget(this);
    auto* cameraDock = new QDockWidget("Camera Feeds", this);
    cameraDock->setWidget(camera_feed_widget_);
    cameraDock->setObjectName("cameraDock");
    addDockWidget(Qt::LeftDockWidgetArea, cameraDock);

    // 3D Canvas (central)
    mocap_canvas_ = new MoCapCanvas(this);
    setCentralWidget(mocap_canvas_);

    // Inspector (right dock)
    inspector_panel_ = new InspectorPanel(this);
    auto* inspectorDock = new QDockWidget("Inspector", this);
    inspectorDock->setWidget(inspector_panel_);
    inspectorDock->setObjectName("inspectorDock");
    addDockWidget(Qt::RightDockWidgetArea, inspectorDock);

    // Timeline (bottom dock)
    timeline_widget_ = new TimelineWidget(this);
    auto* timelineDock = new QDockWidget("Timeline", this);
    timelineDock->setWidget(timeline_widget_);
    timelineDock->setObjectName("timelineDock");
    addDockWidget(Qt::BottomDockWidgetArea, timelineDock);

    // Add toggle actions to View menu
    auto* viewMenu = menuBar()->findChild<QMenu*>("viewMenu");
    if (viewMenu) {
        viewMenu->addAction(cameraDock->toggleViewAction());
        viewMenu->addAction(inspectorDock->toggleViewAction());
        viewMenu->addAction(timelineDock->toggleViewAction());
    }
}

void MainWindow::setupConnections() {
    // Frame broker → processing pipeline
    connect(frame_broker_.get(), &FrameBroker::frameSetReady,
            this, &MainWindow::onFrameSetReady, Qt::QueuedConnection);

    // Canvas → inspector (person/joint selection)
    connect(mocap_canvas_, &MoCapCanvas::personSelected,
            inspector_panel_, &InspectorPanel::onPersonSelected);
    connect(mocap_canvas_, &MoCapCanvas::personSelected,
            this, &MainWindow::onPersonSelected);

    // Timeline transport controls
    connect(timeline_widget_, &TimelineWidget::seekRequested,
            this, &MainWindow::onTimelineSeek);
    connect(timeline_widget_, &TimelineWidget::playRequested,
            this, &MainWindow::onTimelinePlay);
    connect(timeline_widget_, &TimelineWidget::pauseRequested,
            this, &MainWindow::onTimelinePause);
    connect(timeline_widget_, &TimelineWidget::stopRequested,
            this, &MainWindow::onTimelineStop);
    connect(timeline_widget_, &TimelineWidget::recordRequested,
            this, &MainWindow::onTimelineRecord);

    // Playback timer for session replay
    playback_timer_ = new QTimer(this);
    connect(playback_timer_, &QTimer::timeout, this, [this]() {
        if (!playing_ || session_poses3d_.empty()) return;
        double dt = 1.0 / config_.capture.target_fps;
        playback_time_ += dt;

        double max_time = session_poses3d_.back().first;
        if (playback_time_ > max_time) {
            playback_time_ = 0.0;  // loop
        }

        displayFrameAtTime(playback_time_);
        timeline_widget_->setCurrentTime(playback_time_);
    });
}

void MainWindow::loadConfig(const std::string& path) {
    try {
        config_ = AppConfig::load(path);
        spdlog::info("Loaded configuration from {}", path);

        // Apply config to pipeline components
        frame_broker_->setMaxSyncSkewMs(config_.capture.max_sync_skew_ms);
        triangulator_->setMinViews(config_.triangulation.min_views);
        triangulator_->setRansacEnabled(config_.triangulation.ransac_enabled);
        triangulator_->setRansacThreshold(config_.triangulation.ransac_threshold_px);

        temporal_filter_ = std::make_unique<TemporalFilter>(
            TemporalFilter::Type::Butterworth,
            config_.triangulation.filter_cutoff_hz,
            config_.capture.target_fps);

        if (config_.skeleton.ik_solver == "optimisation") {
            skeleton_solver_->setSolverType(SkeletonSolver::SolverType::Optimisation);
        }
        skeleton_solver_->setJointLimitsEnabled(config_.skeleton.joint_limits_enabled);

        // Initialize cameras from config
        initializeCamerasFromConfig();

    } catch (const std::exception& e) {
        spdlog::warn("Could not load {}: {} — using defaults", path, e.what());
    }
}

void MainWindow::initializeCamerasFromConfig() {
    for (const auto& cam_cfg : config_.cameras) {
        std::shared_ptr<ICameraSource> source;

        if (cam_cfg.type == "usb") {
            auto usb = std::make_shared<UsbCameraSource>();
            source = usb;
        } else if (cam_cfg.type == "ip") {
            auto ip = std::make_shared<IpCameraSource>();
            source = ip;
        } else if (cam_cfg.type == "video") {
            auto video = std::make_shared<VideoFileCameraSource>();
            source = video;
        } else {
            spdlog::warn("Unknown camera type: {}", cam_cfg.type);
            continue;
        }

        if (source->open(cam_cfg)) {
            frame_broker_->addCamera(source);
            spdlog::info("Added camera: {} ({})", cam_cfg.id, cam_cfg.type);
        } else {
            spdlog::error("Failed to open camera: {}", cam_cfg.id);
        }
    }
}

// --- Session Management ---

void MainWindow::onNewSession() {
    SessionDialog dialog(SessionDialog::NewSession, this);
    if (dialog.exec() != QDialog::Accepted) return;

    std::string name = dialog.sessionName().toStdString();
    std::string dir = dialog.outputDirectory().toStdString();
    double fps = dialog.recordingFps();

    // Gather camera IDs
    std::vector<std::string> camera_ids;
    for (const auto& cam_cfg : config_.cameras) {
        camera_ids.push_back(cam_cfg.id);
    }

    std::string session_dir = session_manager_->createSession(dir, fps, camera_ids);
    config_.capture.target_fps = fps;

    statusBar()->showMessage("Created session: " +
                             QString::fromStdString(session_dir));
    spdlog::info("Created new session: {}", session_dir);
}

void MainWindow::onOpenSession() {
    SessionDialog dialog(SessionDialog::OpenSession, this);
    if (dialog.exec() != QDialog::Accepted) return;

    std::string path = dialog.selectedSessionPath().toStdString();
    if (path.empty()) return;

    if (session_manager_->openSession(path)) {
        auto meta = session_manager_->metadata();
        statusBar()->showMessage("Opened session: " + QString::fromStdString(path));

        timeline_widget_->setDuration(meta.duration);
        timeline_widget_->setFrameRate(meta.fps);

        loadSessionData();
        spdlog::info("Opened session: {} ({} frames, {:.1f}s)",
                     path, meta.frame_count, meta.duration);
    } else {
        QMessageBox::warning(this, "Error", "Failed to open session directory.");
    }
}

void MainWindow::loadSessionData() {
    if (!session_manager_->isOpen()) return;

    std::string data_dir = session_manager_->dataDir();

    // Load 3D poses if available
    std::string pose3d_path = data_dir + "/pose3d.bin";
    if (std::filesystem::exists(pose3d_path)) {
        session_poses3d_ = BinaryIO::readPose3D(pose3d_path);
        spdlog::info("Loaded {} pose3D frames", session_poses3d_.size());
    }

    // Load skeleton data if available
    std::string skel_path = data_dir + "/skeleton.bin";
    if (std::filesystem::exists(skel_path)) {
        session_skeletons_ = BinaryIO::readSkeleton(skel_path);
        spdlog::info("Loaded {} skeleton frames", session_skeletons_.size());
    }

    // Display first frame
    if (!session_poses3d_.empty()) {
        displayFrameAtTime(0.0);
    }
}

void MainWindow::displayFrameAtTime(double time_seconds) {
    // Find nearest pose3D frame
    if (!session_poses3d_.empty()) {
        auto it = std::lower_bound(
            session_poses3d_.begin(), session_poses3d_.end(), time_seconds,
            [](const auto& frame, double t) { return frame.first < t; });

        if (it == session_poses3d_.end()) --it;
        else if (it != session_poses3d_.begin()) {
            auto prev = std::prev(it);
            if (std::abs(prev->first - time_seconds) < std::abs(it->first - time_seconds)) {
                it = prev;
            }
        }
        mocap_canvas_->onPose3DUpdate(it->second);
        inspector_panel_->onPose3DUpdate(it->second);
    }

    // Find nearest skeleton frame
    if (!session_skeletons_.empty()) {
        auto it = std::lower_bound(
            session_skeletons_.begin(), session_skeletons_.end(), time_seconds,
            [](const auto& frame, double t) { return frame.first < t; });

        if (it == session_skeletons_.end()) --it;
        else if (it != session_skeletons_.begin()) {
            auto prev = std::prev(it);
            if (std::abs(prev->first - time_seconds) < std::abs(it->first - time_seconds)) {
                it = prev;
            }
        }
        mocap_canvas_->onSkeletonUpdate(it->second);
        inspector_panel_->onSkeletonUpdate(it->second);
    }
}

// --- Capture ---

void MainWindow::onStartCapture() {
    if (capturing_) return;

    if (!session_manager_->isOpen()) {
        QMessageBox::information(this, "No Session",
                                 "Please create or open a session first.");
        return;
    }

    // Initialize pose estimator if not ready
    if (!pose_estimator_->isInitialized()) {
        bool ok = pose_estimator_->initialize(
            config_.pose_estimation.model, config_.pose_estimation.device);
        if (!ok) {
            spdlog::warn("Pose estimator initialization failed — "
                         "capture will proceed without pose estimation");
        }
    }

    frame_broker_->start();
    capturing_ = true;
    frame_count_ = 0;
    capture_start_time_ = 0.0;

    statusBar()->showMessage("Capturing...");
    spdlog::info("Capture started");
}

void MainWindow::onStopCapture() {
    if (!capturing_) return;

    frame_broker_->stop();
    capturing_ = false;
    recording_ = false;

    // Close recording writers
    if (raw2d_writer_) { raw2d_writer_->close(); raw2d_writer_.reset(); }
    if (pose3d_writer_) { pose3d_writer_->close(); pose3d_writer_.reset(); }
    if (skeleton_writer_) { skeleton_writer_->close(); skeleton_writer_.reset(); }

    // Update session metadata
    if (session_manager_->isOpen()) {
        session_manager_->setFrameCount(frame_count_);
        session_manager_->setDuration(capture_start_time_);
        session_manager_->saveMetadata();
    }

    statusBar()->showMessage("Capture stopped");
    spdlog::info("Capture stopped ({} frames)", frame_count_);
}

// --- Pipeline Processing ---

void MainWindow::onFrameSetReady(std::shared_ptr<FrameSet> frameSet) {
    frame_count_++;
    processFrameSet(frameSet);
}

void MainWindow::processFrameSet(std::shared_ptr<FrameSet> frameSet) {
    // Update camera feeds
    camera_feed_widget_->onFrameSet(frameSet);

    // Run 2D pose estimation on each camera
    std::vector<std::pair<std::string, std::vector<Raw2DPose>>> all_poses;
    for (const auto& frame : frameSet->frames) {
        if (pose_estimator_->isInitialized()) {
            auto poses = pose_estimator_->estimate(frame.image);
            all_poses.emplace_back(frame.camera_id, poses);
        }
    }

    // Update 2D overlays
    if (!all_poses.empty()) {
        camera_feed_widget_->onPoses2D(all_poses);
    }

    // Track persons across cameras
    auto tracked = person_tracker_->update(all_poses, frameSet->timestamp);

    // Triangulate 3D positions
    auto poses3d = triangulator_->triangulate(tracked, frameSet->timestamp);

    // Apply temporal filtering to each marker
    // (In a full implementation, maintain per-marker filter state)

    mocap_canvas_->onPose3DUpdate(poses3d);
    inspector_panel_->onPose3DUpdate(poses3d);

    // Solve skeleton
    std::vector<SkeletonPose> skeletons;
    for (const auto& p3d : poses3d) {
        skeletons.push_back(skeleton_solver_->solve(p3d));
    }
    mocap_canvas_->onSkeletonUpdate(skeletons);
    inspector_panel_->onSkeletonUpdate(skeletons);

    // Update timeline
    capture_start_time_ = frameSet->timestamp;
    timeline_widget_->setCurrentTime(frameSet->timestamp);

    // Update person count in status bar
    status_persons_->setText(QString("Persons: %1").arg(person_tracker_->activePersonCount()));
}

// --- Timeline Controls ---

void MainWindow::onTimelineSeek(double time_seconds) {
    if (!session_poses3d_.empty()) {
        displayFrameAtTime(time_seconds);
        playback_time_ = time_seconds;
    }
}

void MainWindow::onTimelinePlay() {
    if (capturing_) return;  // don't play during live capture

    if (!session_poses3d_.empty()) {
        playing_ = true;
        double interval_ms = 1000.0 / config_.capture.target_fps;
        playback_timer_->start(static_cast<int>(interval_ms));
    }
}

void MainWindow::onTimelinePause() {
    playing_ = false;
    playback_timer_->stop();
}

void MainWindow::onTimelineStop() {
    playing_ = false;
    playback_timer_->stop();
    playback_time_ = 0.0;
    timeline_widget_->setCurrentTime(0.0);

    if (!session_poses3d_.empty()) {
        displayFrameAtTime(0.0);
    }
}

void MainWindow::onTimelineRecord() {
    if (!capturing_) {
        onStartCapture();
    }

    if (!session_manager_->isOpen()) return;

    recording_ = !recording_;
    if (recording_) {
        std::string data_dir = session_manager_->dataDir();
        raw2d_writer_ = std::make_unique<BinaryIO::StreamWriter>(
            data_dir + "/raw2d.bin", 1);
        pose3d_writer_ = std::make_unique<BinaryIO::StreamWriter>(
            data_dir + "/pose3d.bin", 2);
        skeleton_writer_ = std::make_unique<BinaryIO::StreamWriter>(
            data_dir + "/skeleton.bin", 3);
        spdlog::info("Recording started");
    } else {
        if (raw2d_writer_) { raw2d_writer_->close(); raw2d_writer_.reset(); }
        if (pose3d_writer_) { pose3d_writer_->close(); pose3d_writer_.reset(); }
        if (skeleton_writer_) { skeleton_writer_->close(); skeleton_writer_.reset(); }
        spdlog::info("Recording stopped");
    }
}

void MainWindow::onPersonSelected(int global_person_id) {
    inspector_panel_->onPersonSelected(global_person_id);
    statusBar()->showMessage(global_person_id >= 0
        ? QString("Selected Person %1").arg(global_person_id)
        : "Selection cleared");
}

// --- Export ---

void MainWindow::onExport() {
    if (!session_manager_->isOpen()) {
        QMessageBox::information(this, "No Session",
                                 "Please open a session to export data from.");
        return;
    }

    auto meta = session_manager_->metadata();
    ExportDialog dialog(meta.frame_count, meta.fps, this);
    if (dialog.exec() != QDialog::Accepted) return;

    runExport(dialog.format().toStdString(),
              dialog.exportL1(), dialog.exportL2(), dialog.exportL3(),
              dialog.startFrame(), dialog.endFrame(),
              dialog.outputDirectory().toStdString());
}

void MainWindow::runExport(const std::string& format, bool l1, bool l2, bool l3,
                            int start_frame, int end_frame,
                            const std::string& output_dir) {
    spdlog::info("Exporting {} data to {}", format, output_dir);
    std::filesystem::create_directories(output_dir);

    auto meta = session_manager_->metadata();
    std::string data_dir = session_manager_->dataDir();

    // Load data for export
    auto poses3d = BinaryIO::readPose3D(data_dir + "/pose3d.bin");
    auto skeletons = BinaryIO::readSkeleton(data_dir + "/skeleton.bin");

    // Trim to frame range
    auto trimFrames = [&](auto& frames) {
        if (start_frame > 0 || end_frame < static_cast<int>(frames.size()) - 1) {
            int s = std::min(start_frame, static_cast<int>(frames.size()));
            int e = std::min(end_frame + 1, static_cast<int>(frames.size()));
            frames.erase(frames.begin() + e, frames.end());
            frames.erase(frames.begin(), frames.begin() + s);
        }
    };

    if (format == "CSV") {
        if (l2 && !poses3d.empty()) {
            trimFrames(poses3d);
            CsvExporter::exportPose3D(output_dir + "/pose3d.csv", poses3d);
        }
        if (l3 && !skeletons.empty()) {
            trimFrames(skeletons);
            CsvExporter::exportSkeleton(output_dir + "/skeleton.csv", skeletons);
        }
    } else if (format == "JSON") {
        if (l2 && !poses3d.empty()) {
            trimFrames(poses3d);
            JsonExporter::exportPose3D(output_dir + "/pose3d.json", poses3d);
        }
        if (l3 && !skeletons.empty()) {
            trimFrames(skeletons);
            JsonExporter::exportSkeleton(output_dir + "/skeleton.json", skeletons);
        }
    } else if (format == "C3D") {
        if (!poses3d.empty()) {
            trimFrames(poses3d);
            C3dExporter::exportPose3D(output_dir + "/mocap.c3d", poses3d, meta.fps);
        }
    } else if (format == "BVH") {
        if (!skeletons.empty()) {
            trimFrames(skeletons);
            auto skel_def = SkeletonDefinition::defaultBody25();
            BvhExporter::exportSkeleton(output_dir + "/mocap.bvh",
                                        skel_def, skeletons, meta.fps);
        }
    } else if (format == "FBX") {
        if (!skeletons.empty()) {
            trimFrames(skeletons);
            auto skel_def = SkeletonDefinition::defaultBody25();
            FbxExporter::exportSkeleton(output_dir + "/mocap.fbx",
                                        skel_def, skeletons, meta.fps);
        }
    } else if (format == "USD") {
        if (!skeletons.empty()) {
            trimFrames(skeletons);
            auto skel_def = SkeletonDefinition::defaultBody25();
            UsdExporter::exportSkeleton(output_dir + "/mocap.usda",
                                        skel_def, skeletons, meta.fps);
        }
    }

    statusBar()->showMessage("Export complete: " + QString::fromStdString(output_dir));
    spdlog::info("Export complete: {}", output_dir);
}

// --- Camera Setup & Calibration ---

void MainWindow::onCameraSetup() {
    QMessageBox::information(this, "Camera Setup",
        "Camera configuration is managed via config.yaml.\n\n"
        "Add camera entries under the 'cameras' section:\n"
        "  - id: cam0\n"
        "    type: usb\n"
        "    device_index: 0\n"
        "    resolution: [1920, 1080]");
}

void MainWindow::onCalibrate() {
    // Gather active camera sources
    std::vector<std::shared_ptr<ICameraSource>> sources;
    // Note: in production, collect sources from frame_broker_

    if (sources.empty()) {
        QMessageBox::information(this, "No Cameras",
                                 "Please add cameras in config.yaml before calibrating.");
        return;
    }

    CalibrationWizard wizard(sources, this);
    if (wizard.exec() == QDialog::Accepted) {
        auto intrinsics = wizard.intrinsicsResults();
        auto extrinsics = wizard.extrinsicsResults();

        if (session_manager_->isOpen()) {
            std::string calib_dir = session_manager_->calibrationDir();
            // Save calibration files
            for (size_t i = 0; i < intrinsics.size(); ++i) {
                std::string path = calib_dir + "/camera_" +
                                   std::to_string(i) + "_intrinsics.yaml";
                intrinsics[i].saveToYaml(path);
            }
            for (size_t i = 0; i < extrinsics.size(); ++i) {
                std::string path = calib_dir + "/camera_" +
                                   std::to_string(i) + "_extrinsics.json";
                extrinsics[i].saveToJson(path);
            }
            spdlog::info("Saved calibration to {}", calib_dir);
        }
    }
}

void MainWindow::onAbout() {
    QMessageBox::about(this, "About MoCap Studio",
        "MoCap Studio v0.1.0\n\n"
        "Markerless multi-camera motion capture.\n\n"
        "Real-time 2D pose estimation, 3D triangulation,\n"
        "and skeletal IK fitting.\n\n"
        "Pipeline: Capture → Pose → Triangulate → Skeleton → Export");
}

void MainWindow::updateStatusBar() {
    status_cameras_->setText(QString("Cameras: %1").arg(frame_broker_->cameraCount()));
    status_persons_->setText(QString("Persons: %1").arg(person_tracker_->activePersonCount()));

    current_fps_ = frame_count_;
    frame_count_ = 0;
    status_fps_->setText(QString("FPS: %1").arg(static_cast<int>(current_fps_)));
}

}  // namespace mocap
