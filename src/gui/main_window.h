#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QTimer>
#include <memory>

#include "core/config.h"
#include "capture/frame_broker.h"
#include "pose/onnx_pose_estimator.h"
#include "pose/person_tracker.h"
#include "triangulation/triangulator.h"
#include "triangulation/temporal_filter.h"
#include "skeleton/skeleton_solver.h"
#include "storage/session_manager.h"

namespace mocap {

class CameraFeedWidget;
class MoCapCanvas;
class TimelineWidget;
class InspectorPanel;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void onNewSession();
    void onOpenSession();
    void onStartCapture();
    void onStopCapture();
    void onExport();
    void onCameraSetup();
    void onCalibrate();
    void onAbout();

    void onFrameSetReady(std::shared_ptr<FrameSet> frameSet);
    void updateStatusBar();

private:
    void setupMenuBar();
    void setupStatusBar();
    void setupDockWidgets();
    void setupConnections();
    void loadConfig();

    void processFrameSet(std::shared_ptr<FrameSet> frameSet);

    // Pipeline components
    std::shared_ptr<FrameBroker> frame_broker_;
    std::unique_ptr<OnnxPoseEstimator> pose_estimator_;
    std::unique_ptr<PersonTracker> person_tracker_;
    std::unique_ptr<Triangulator> triangulator_;
    std::unique_ptr<SkeletonSolver> skeleton_solver_;
    std::unique_ptr<SessionManager> session_manager_;

    // GUI widgets
    CameraFeedWidget* camera_feed_widget_ = nullptr;
    MoCapCanvas* mocap_canvas_ = nullptr;
    TimelineWidget* timeline_widget_ = nullptr;
    InspectorPanel* inspector_panel_ = nullptr;

    // Status bar labels
    QLabel* status_cameras_ = nullptr;
    QLabel* status_persons_ = nullptr;
    QLabel* status_fps_ = nullptr;

    // FPS tracking
    QTimer* fps_timer_ = nullptr;
    int frame_count_ = 0;
    double current_fps_ = 0.0;

    // Configuration
    AppConfig config_;
    bool capturing_ = false;
};

}  // namespace mocap
