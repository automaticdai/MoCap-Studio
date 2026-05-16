#pragma once

#include <QWizard>
#include <QWizardPage>
#include <QListWidget>
#include <QLabel>
#include <QProgressBar>
#include <QSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QTableWidget>
#include <QWidget>
#include <opencv2/core.hpp>
#include <vector>
#include <memory>
#include "capture/icamera_source.h"
#include "core/camera_intrinsics.h"
#include "core/config.h"
#include "core/intrinsics_calibrator.h"

namespace mocap {

// Tiny inline histogram widget for per-frame reprojection errors.
// Draws bars with a horizontal line at the user-supplied threshold so
// "good" vs "bad" frames are visually obvious.
class ReprojErrorHistogram : public QWidget {
    Q_OBJECT
public:
    explicit ReprojErrorHistogram(QWidget* parent = nullptr);
    void setData(const std::vector<double>& errors_px, double threshold_px = 1.0);

protected:
    void paintEvent(QPaintEvent* event) override;
    QSize sizeHint() const override { return QSize(280, 60); }

private:
    std::vector<double> errors_;
    double threshold_ = 1.0;
};

class CalibrationWizard : public QWizard {
    Q_OBJECT
public:
    explicit CalibrationWizard(
        const std::vector<CameraConfig>& camera_configs,
        QWidget* parent = nullptr);
    ~CalibrationWizard() override;

    std::vector<CameraIntrinsics> intrinsicsResults() const;
    std::vector<CameraExtrinsics> extrinsicsResults() const;  // empty in Phase 1

    // --- Shared state, mutated by individual pages ---
    const std::vector<CameraConfig>& cameraConfigs() const { return camera_configs_; }
    ICameraSource* cameraSource(int config_index);

    CheckerboardSpec& boardSpec() { return board_; }
    const CheckerboardSpec& boardSpec() const { return board_; }

    void setSelectedCameraIndices(const std::vector<int>& indices);
    const std::vector<int>& selectedCameraIndices() const { return selected_indices_; }

    // For each entry in selected_indices_, the corners detected in each view
    // and the image size at which they were captured.
    std::vector<std::vector<cv::Point2f>>& cornersForSlot(int slot);
    cv::Size& imageSizeForSlot(int slot);
    int numSlots() const { return static_cast<int>(selected_indices_.size()); }

    void setResultForSlot(int slot, const IntrinsicsResult& result);
    const IntrinsicsResult& resultForSlot(int slot) const;

private:
    std::vector<CameraConfig> camera_configs_;
    std::vector<std::shared_ptr<ICameraSource>> sources_;  // one per camera_configs_ entry
    std::vector<int> selected_indices_;
    CheckerboardSpec board_;

    // Indexed by slot (position in selected_indices_), not by camera_configs_ index.
    std::vector<std::vector<std::vector<cv::Point2f>>> per_slot_corners_;
    std::vector<cv::Size> per_slot_image_size_;
    std::vector<IntrinsicsResult> per_slot_results_;
};

// Page 1: Select cameras
class CameraSelectionPage : public QWizardPage {
    Q_OBJECT
public:
    explicit CameraSelectionPage(QWidget* parent = nullptr);
    void initializePage() override;
    bool validatePage() override;
    bool isComplete() const override;

private:
    QListWidget* camera_list_;
};

// Page 2: Capture calibration frames
class CaptureCalibrationPage : public QWizardPage {
    Q_OBJECT
public:
    explicit CaptureCalibrationPage(QWidget* parent = nullptr);
    void initializePage() override;
    bool isComplete() const override;

private slots:
    void onCaptureFrame();
    void onBoardParamChanged();

private:
    void refreshCounts();
    void refreshBoardSpecFromUi();

    QLabel* preview_label_;
    QLabel* status_label_;
    QListWidget* count_list_;
    QPushButton* capture_btn_;
    QSpinBox* board_cols_spin_;
    QSpinBox* board_rows_spin_;
    QDoubleSpinBox* square_size_spin_;
    QComboBox* board_type_combo_;

    int min_frames_required_ = 8;
};

// Page 3: Compute calibration
class ComputeCalibrationPage : public QWizardPage {
    Q_OBJECT
public:
    explicit ComputeCalibrationPage(QWidget* parent = nullptr);
    void initializePage() override;
    bool isComplete() const override;

private slots:
    void onRunCalibration();

private:
    QProgressBar* progress_bar_;
    QLabel* status_label_;
    QPushButton* run_btn_;
    bool calibration_done_ = false;
};

// Page 4: Review results
class ReviewCalibrationPage : public QWizardPage {
    Q_OBJECT
public:
    explicit ReviewCalibrationPage(QWidget* parent = nullptr);
    void initializePage() override;

private:
    QTableWidget* results_table_;
    QLabel* summary_label_;
    QVBoxLayout* histograms_layout_;
};

}  // namespace mocap
