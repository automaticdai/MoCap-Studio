#pragma once

#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QPixmap>
#include <unordered_map>
#include <memory>
#include "core/types.h"

namespace mocap {

class CameraFeedWidget : public QWidget {
    Q_OBJECT
public:
    explicit CameraFeedWidget(QWidget* parent = nullptr);

public slots:
    void onFrameSet(std::shared_ptr<FrameSet> frameSet);
    void onPoses2D(const std::vector<std::pair<std::string, std::vector<Raw2DPose>>>& poses);

    void setKeypointOverlayEnabled(bool enabled);
    bool keypointOverlayEnabled() const;

private:
    QGridLayout* grid_layout_;
    std::unordered_map<std::string, QLabel*> camera_labels_;
    std::unordered_map<std::string, std::vector<Raw2DPose>> latest_poses_;
    bool show_keypoint_overlay_ = true;
    int max_columns_ = 1;

    QLabel* getOrCreateLabel(const std::string& camera_id);
    void removeLabel(const std::string& camera_id);

    static QPixmap matToPixmap(const cv::Mat& mat);
    static void drawKeypointsOverlay(QPixmap& pixmap, const std::vector<Raw2DPose>& poses);

    // BODY_25 skeleton connections for overlay drawing
    static const std::vector<std::pair<int, int>> SKELETON_CONNECTIONS;
};

}  // namespace mocap
