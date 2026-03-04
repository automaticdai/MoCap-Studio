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
#include <vector>
#include <memory>
#include "capture/icamera_source.h"
#include "core/camera_intrinsics.h"

namespace mocap {

class CalibrationWizard : public QWizard {
    Q_OBJECT
public:
    explicit CalibrationWizard(
        const std::vector<std::shared_ptr<ICameraSource>>& cameras,
        QWidget* parent = nullptr);

    std::vector<CameraIntrinsics> intrinsicsResults() const;
    std::vector<CameraExtrinsics> extrinsicsResults() const;

private:
    std::vector<std::shared_ptr<ICameraSource>> cameras_;
    std::vector<CameraIntrinsics> intrinsics_results_;
    std::vector<CameraExtrinsics> extrinsics_results_;
};

// Page 1: Select cameras
class CameraSelectionPage : public QWizardPage {
    Q_OBJECT
public:
    explicit CameraSelectionPage(
        const std::vector<std::shared_ptr<ICameraSource>>& cameras,
        QWidget* parent = nullptr);
    bool isComplete() const override;
    std::vector<int> selectedCameraIndices() const;

private:
    QListWidget* camera_list_;
};

// Page 2: Capture calibration frames
class CaptureCalibrationPage : public QWizardPage {
    Q_OBJECT
public:
    explicit CaptureCalibrationPage(QWidget* parent = nullptr);
    void initializePage() override;

private slots:
    void onCaptureFrame();

private:
    QLabel* preview_label_;
    QLabel* frame_count_label_;
    QPushButton* capture_btn_;
    QSpinBox* board_cols_spin_;
    QSpinBox* board_rows_spin_;
    QDoubleSpinBox* square_size_spin_;
    QComboBox* board_type_combo_;
    int captured_frames_ = 0;
    int target_frames_ = 20;
};

// Page 3: Compute calibration
class ComputeCalibrationPage : public QWizardPage {
    Q_OBJECT
public:
    explicit ComputeCalibrationPage(QWidget* parent = nullptr);
    void initializePage() override;

private slots:
    void onRunCalibration();

private:
    QProgressBar* progress_bar_;
    QLabel* status_label_;
    QPushButton* run_btn_;
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
};

}  // namespace mocap
