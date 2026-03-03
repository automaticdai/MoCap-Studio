#include "gui/main_window.h"
#include "gui/camera_feed_widget.h"
#include "gui/mocap_canvas.h"
#include "gui/timeline_widget.h"
#include "gui/inspector_panel.h"
#include "gui/session_dialog.h"
#include "gui/calibration_wizard.h"
#include "gui/export_dialog.h"

#include <QMenuBar>
#include <QStatusBar>
#include <QDockWidget>
#include <QMessageBox>
#include <QFileDialog>
#include <spdlog/spdlog.h>

namespace mocap {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("MoCap Studio");
    resize(1600, 900);

    // Initialize pipeline components
    frame_broker_ = std::make_shared<FrameBroker>(this);
    pose_estimator_ = std::make_unique<OnnxPoseEstimator>();
    person_tracker_ = std::make_unique<PersonTracker>();
    triangulator_ = std::make_unique<Triangulator>();
    skeleton_solver_ = std::make_unique<SkeletonSolver>();
    session_manager_ = std::make_unique<SessionManager>();

    // Set default skeleton
    skeleton_solver_->setSkeletonDefinition(SkeletonDefinition::defaultBody25());

    setupMenuBar();
    setupStatusBar();
    setupDockWidgets();
    setupConnections();
    loadConfig();
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
    // Dock visibility toggles will be added after dock creation

    auto* exportMenu = menuBar()->addMenu("&Export");
    exportMenu->addAction("&Export Data...", this, &MainWindow::onExport);

    auto* helpMenu = menuBar()->addMenu("&Help");
    helpMenu->addAction("&About", this, &MainWindow::onAbout);

    // Store view menu for dock toggle actions
    viewMenu->setObjectName("viewMenu");
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
    connect(frame_broker_.get(), &FrameBroker::frameSetReady,
            this, &MainWindow::onFrameSetReady, Qt::QueuedConnection);
}

void MainWindow::loadConfig() {
    try {
        config_ = AppConfig::load("config.yaml");
        spdlog::info("Loaded configuration from config.yaml");

        // Apply config
        frame_broker_->setMaxSyncSkewMs(config_.capture.max_sync_skew_ms);
        triangulator_->setMinViews(config_.triangulation.min_views);
        triangulator_->setRansacEnabled(config_.triangulation.ransac_enabled);
        triangulator_->setRansacThreshold(config_.triangulation.ransac_threshold_px);

        if (config_.skeleton.ik_solver == "optimisation") {
            skeleton_solver_->setSolverType(SkeletonSolver::SolverType::Optimisation);
        }
        skeleton_solver_->setJointLimitsEnabled(config_.skeleton.joint_limits_enabled);

    } catch (const std::exception& e) {
        spdlog::warn("Could not load config.yaml: {} — using defaults", e.what());
    }
}

void MainWindow::onNewSession() {
    SessionDialog dialog(SessionDialog::Mode::New, this);
    if (dialog.exec() == QDialog::Accepted) {
        auto result = dialog.result();
        session_manager_->createSession(result.directory, result.fps, {});
        statusBar()->showMessage("Created session: " + QString::fromStdString(session_manager_->sessionDir()));
    }
}

void MainWindow::onOpenSession() {
    QString dir = QFileDialog::getExistingDirectory(this, "Open Session Directory");
    if (dir.isEmpty()) return;

    if (session_manager_->openSession(dir.toStdString())) {
        statusBar()->showMessage("Opened session: " + dir);
        auto meta = session_manager_->metadata();
        timeline_widget_->setDuration(meta.duration);
        timeline_widget_->setFrameRate(meta.fps);
    } else {
        QMessageBox::warning(this, "Error", "Failed to open session directory");
    }
}

void MainWindow::onStartCapture() {
    if (capturing_) return;

    if (!session_manager_->isOpen()) {
        QMessageBox::information(this, "No Session",
                                 "Please create or open a session first.");
        return;
    }

    frame_broker_->start();
    capturing_ = true;
    frame_count_ = 0;
    statusBar()->showMessage("Capturing...");
    spdlog::info("Capture started");
}

void MainWindow::onStopCapture() {
    if (!capturing_) return;

    frame_broker_->stop();
    capturing_ = false;
    statusBar()->showMessage("Capture stopped");
    spdlog::info("Capture stopped");
}

void MainWindow::onExport() {
    if (!session_manager_->isOpen()) {
        QMessageBox::information(this, "No Session",
                                 "Please open a session to export data from.");
        return;
    }

    ExportDialog dialog(session_manager_->exportDir(), this);
    dialog.exec();
}

void MainWindow::onCameraSetup() {
    QMessageBox::information(this, "Camera Setup",
                             "Camera setup dialog — configure cameras in config.yaml");
}

void MainWindow::onCalibrate() {
    if (frame_broker_->cameraCount() == 0) {
        QMessageBox::information(this, "No Cameras",
                                 "Please add cameras before calibrating.");
        return;
    }

    CalibrationWizard wizard(this);
    wizard.exec();
}

void MainWindow::onAbout() {
    QMessageBox::about(this, "About MoCap Studio",
        "MoCap Studio v0.1.0\n\n"
        "Markerless multi-camera motion capture.\n\n"
        "Real-time 2D pose estimation, 3D triangulation,\n"
        "and skeletal IK fitting.");
}

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
        auto poses = pose_estimator_->estimate(frame.image);
        all_poses.emplace_back(frame.camera_id, poses);
    }

    // Update 2D overlays
    camera_feed_widget_->onPoses2D(all_poses);

    // Track persons
    auto tracked = person_tracker_->update(all_poses, frameSet->timestamp);

    // Triangulate
    auto poses3d = triangulator_->triangulate(tracked, frameSet->timestamp);
    mocap_canvas_->onPose3DUpdate(poses3d);

    // Solve skeleton
    std::vector<SkeletonPose> skeletons;
    for (const auto& p3d : poses3d) {
        skeletons.push_back(skeleton_solver_->solve(p3d));
    }
    mocap_canvas_->onSkeletonUpdate(skeletons);
    inspector_panel_->onSkeletonUpdate(skeletons);

    // Update timeline
    timeline_widget_->setCurrentTime(frameSet->timestamp);
}

void MainWindow::updateStatusBar() {
    status_cameras_->setText(QString("Cameras: %1").arg(frame_broker_->cameraCount()));
    status_persons_->setText(QString("Persons: %1").arg(person_tracker_->activePersonCount()));

    current_fps_ = frame_count_;
    frame_count_ = 0;
    status_fps_->setText(QString("FPS: %1").arg(static_cast<int>(current_fps_)));
}

}  // namespace mocap
