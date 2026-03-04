#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QTimer>
#include <QThread>
#include <memory>
#include <atomic>

#include "core/config.h"
#include "capture/frame_broker.h"
#include "pose/onnx_pose_estimator.h"
#include "pose/person_tracker.h"
#include "triangulation/triangulator.h"
#include "triangulation/temporal_filter.h"
#include "skeleton/skeleton_solver.h"
#include "storage/session_manager.h"
#include "storage/binary_io.h"

namespace mocap {

class CameraFeedWidget;
class MoCapCanvas;
class TimelineWidget;
class InspectorPanel;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(const std::string& config_path = "config.yaml",
                        QWidget* parent = nullptr);
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
    void onTimelineSeek(double time_seconds);
    void onTimelinePlay();
    void onTimelinePause();
    void onTimelineStop();
    void onTimelineRecord();
    void onPersonSelected(int global_person_id);
    void updateStatusBar();

private:
    void setupMenuBar();
    void setupStatusBar();
    void setupDockWidgets();
    void setupConnections();
    void loadConfig(const std::string& path);

    void processFrameSet(std::shared_ptr<FrameSet> frameSet);
    void initializeCamerasFromConfig();
    void runExport(const std::string& format, bool l1, bool l2, bool l3,
                   int start_frame, int end_frame, const std::string& output_dir);

    // Session playback
    void loadSessionData();
    void displayFrameAtTime(double time_seconds);

    // Pipeline components
    std::shared_ptr<FrameBroker> frame_broker_;
    std::unique_ptr<OnnxPoseEstimator> pose_estimator_;
    std::unique_ptr<PersonTracker> person_tracker_;
    std::unique_ptr<Triangulator> triangulator_;
    std::unique_ptr<TemporalFilter> temporal_filter_;
    std::unique_ptr<SkeletonSolver> skeleton_solver_;
    std::unique_ptr<SessionManager> session_manager_;

    // Live recording writers
    std::unique_ptr<BinaryIO::StreamWriter> raw2d_writer_;
    std::unique_ptr<BinaryIO::StreamWriter> pose3d_writer_;
    std::unique_ptr<BinaryIO::StreamWriter> skeleton_writer_;

    // Session playback data (loaded from files)
    std::vector<std::pair<double, std::vector<Pose3D>>> session_poses3d_;
    std::vector<std::pair<double, std::vector<SkeletonPose>>> session_skeletons_;

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

    // Playback timer
    QTimer* playback_timer_ = nullptr;
    double playback_time_ = 0.0;
    bool playing_ = false;

    // Configuration
    AppConfig config_;
    bool capturing_ = false;
    bool recording_ = false;
    double capture_start_time_ = 0.0;
};

}  // namespace mocap
