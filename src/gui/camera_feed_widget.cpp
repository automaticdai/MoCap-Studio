#include "gui/camera_feed_widget.h"
#include <QPainter>
#include <QImage>
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace mocap {

namespace {
// Painting into a stable viewport avoids QLabel's pixmap size hint feeding
// back into the dock layout (each frame previously added the label border).
class FeedLabel : public QLabel {
public:
    using QLabel::QLabel;
    QPixmap image;
    QString camera_id;
    QSize sizeHint() const override { return {320, 180}; }
    QSize minimumSizeHint() const override { return {160, 100}; }
    void paintEvent(QPaintEvent* event) override {
        QLabel::paintEvent(event);
        if (image.isNull()) return;
        QPainter painter(this);
        painter.setRenderHint(QPainter::SmoothPixmapTransform);
        const QRect bounds = contentsRect().adjusted(1, 1, -1, -1);
        QSize scaled = image.size();
        scaled.scale(bounds.size(), Qt::KeepAspectRatio);
        QRect target(QPoint(), scaled);
        target.moveCenter(bounds.center());
        painter.drawPixmap(target, image);
        painter.fillRect(8, 8, painter.fontMetrics().horizontalAdvance(camera_id) + 12,
                         24, QColor(0, 0, 0, 160));
        painter.setPen(Qt::white);
        painter.drawText(14, 25, camera_id);
    }
};
}

const std::vector<std::pair<int, int>> CameraFeedWidget::SKELETON_CONNECTIONS = {
    {0, 1}, {1, 2}, {2, 3}, {3, 4}, {1, 5}, {5, 6}, {6, 7},
    {1, 8}, {8, 9}, {9, 10}, {10, 11}, {8, 12}, {12, 13}, {13, 14},
    {0, 15}, {0, 16}, {15, 17}, {16, 18},
    {14, 19}, {14, 20}, {14, 21}, {11, 22}, {11, 23}, {11, 24}
};

CameraFeedWidget::CameraFeedWidget(QWidget* parent)
    : QWidget(parent)
{
    grid_layout_ = new QGridLayout(this);
    grid_layout_->setSpacing(4);
    grid_layout_->setContentsMargins(4, 4, 4, 4);
    setLayout(grid_layout_);
    setMinimumWidth(250);
}

void CameraFeedWidget::onFrameSet(std::shared_ptr<FrameSet> frameSet) {
    if (!frameSet) return;

    for (const auto& frame : frameSet->frames) {
        if (frame.image.empty()) continue;
        latest_images_[frame.camera_id] = matToPixmap(frame.image);
        refreshPreview(frame.camera_id);
    }
}

void CameraFeedWidget::onPoses2D(
    const std::vector<std::pair<std::string, std::vector<Raw2DPose>>>& poses)
{
    for (const auto& [cam_id, cam_poses] : poses) {
        latest_poses_[cam_id] = cam_poses;
        refreshPreview(cam_id);
    }
}

void CameraFeedWidget::setKeypointOverlayEnabled(bool enabled) {
    show_keypoint_overlay_ = enabled;
    for (const auto& entry : latest_images_) refreshPreview(entry.first);
}

void CameraFeedWidget::refreshPreview(const std::string& camera_id) {
    auto* label = static_cast<FeedLabel*>(getOrCreateLabel(camera_id));
    auto image = latest_images_.find(camera_id);
    if (image == latest_images_.end()) return;
    QPixmap pixmap = image->second;
    const auto poses = latest_poses_.find(camera_id);
    if (show_keypoint_overlay_ && poses != latest_poses_.end())
        drawKeypointsOverlay(pixmap, poses->second);
    label->setText({});
    label->image = std::move(pixmap);
    label->update();
}

void CameraFeedWidget::setCameras(const std::vector<std::string>& camera_ids) {
    while (!camera_labels_.empty()) {
        const auto id = camera_labels_.begin()->first;
        removeLabel(id);
    }
    for (const auto& id : camera_ids) setCameraStatus(id, "Connecting…");
    if (camera_ids.empty()) {
        setToolTip("Add a camera from Cameras → Camera Setup.");
    } else setToolTip({});
}

void CameraFeedWidget::setCameraStatus(const std::string& camera_id, const QString& status) {
    latest_images_.erase(camera_id);
    latest_poses_.erase(camera_id);
    auto* label = static_cast<FeedLabel*>(getOrCreateLabel(camera_id));
    label->image = {};
    label->setText(QString::fromStdString(camera_id) + "\n" + status);
    label->update();
}

void CameraFeedWidget::clearOverlays() {
    latest_poses_.clear();
    for (const auto& entry : latest_images_) refreshPreview(entry.first);
}

bool CameraFeedWidget::keypointOverlayEnabled() const {
    return show_keypoint_overlay_;
}

QLabel* CameraFeedWidget::getOrCreateLabel(const std::string& camera_id) {
    auto it = camera_labels_.find(camera_id);
    if (it != camera_labels_.end()) {
        return it->second;
    }

    auto* label = new FeedLabel(this);
    label->camera_id = QString::fromStdString(camera_id);
    label->setObjectName("cameraPreview_" + label->camera_id);
    label->setAlignment(Qt::AlignCenter);
    label->setMinimumSize(160, 100);
    label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    label->setStyleSheet("QLabel { background-color: #1a1a2e; border: 1px solid #333; }");
    label->setText(QString::fromStdString(camera_id));

    int row = static_cast<int>(camera_labels_.size()) / max_columns_;
    int col = static_cast<int>(camera_labels_.size()) % max_columns_;
    grid_layout_->addWidget(label, row, col);

    camera_labels_[camera_id] = label;
    return label;
}

void CameraFeedWidget::removeLabel(const std::string& camera_id) {
    auto it = camera_labels_.find(camera_id);
    if (it == camera_labels_.end()) return;

    grid_layout_->removeWidget(it->second);
    delete it->second;
    camera_labels_.erase(it);
    latest_poses_.erase(camera_id);
    latest_images_.erase(camera_id);
}

QPixmap CameraFeedWidget::matToPixmap(const cv::Mat& mat) {
    if (mat.empty()) return QPixmap();

    cv::Mat rgb;
    if (mat.channels() == 3) {
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    } else if (mat.channels() == 1) {
        cv::cvtColor(mat, rgb, cv::COLOR_GRAY2RGB);
    } else {
        rgb = mat;
    }

    QImage img(rgb.data, rgb.cols, rgb.rows,
               static_cast<int>(rgb.step), QImage::Format_RGB888);
    return QPixmap::fromImage(img.copy());
}

void CameraFeedWidget::drawKeypointsOverlay(QPixmap& pixmap, const std::vector<Raw2DPose>& poses) {
    if (pixmap.isNull() || poses.empty()) return;

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);

    // Color palette for different persons
    static const QColor colors[] = {
        QColor(0, 255, 128), QColor(255, 128, 0), QColor(128, 0, 255),
        QColor(255, 255, 0), QColor(0, 200, 255), QColor(255, 0, 128)
    };

    for (size_t pi = 0; pi < poses.size(); ++pi) {
        const auto& pose = poses[pi];
        QColor color = colors[pi % 6];
        QPen linePen(color, 2);
        QPen dotPen(color, 1);

        // Draw skeleton connections
        painter.setPen(linePen);
        for (const auto& [a, b] : SKELETON_CONNECTIONS) {
            if (a >= static_cast<int>(pose.keypoints.size()) ||
                b >= static_cast<int>(pose.keypoints.size())) continue;

            const auto& kpA = pose.keypoints[a];
            const auto& kpB = pose.keypoints[b];

            if (kpA.conf < 0.3f || kpB.conf < 0.3f) continue;

            painter.drawLine(
                QPointF(kpA.x, kpA.y),
                QPointF(kpB.x, kpB.y)
            );
        }

        // Draw keypoints
        for (const auto& kp : pose.keypoints) {
            if (kp.conf < 0.3f) continue;

            int alpha = static_cast<int>(std::clamp(kp.conf, 0.0f, 1.0f) * 255);
            QColor kpColor = color;
            kpColor.setAlpha(alpha);

            painter.setPen(Qt::NoPen);
            painter.setBrush(kpColor);
            painter.drawEllipse(QPointF(kp.x, kp.y), 3.0, 3.0);
        }

        // Draw bounding box
        if (pose.bbox.width > 0 && pose.bbox.height > 0) {
            QPen bboxPen(color, 1, Qt::DashLine);
            painter.setPen(bboxPen);
            painter.setBrush(Qt::NoBrush);
            painter.drawRect(QRectF(pose.bbox.x, pose.bbox.y,
                                    pose.bbox.width, pose.bbox.height));
        }
    }
}

}  // namespace mocap
