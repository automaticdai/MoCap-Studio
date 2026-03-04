#include "gui/calibration_wizard.h"
#include <QCheckBox>

namespace mocap {

// --- CalibrationWizard ---

CalibrationWizard::CalibrationWizard(
    const std::vector<std::shared_ptr<ICameraSource>>& cameras,
    QWidget* parent)
    : QWizard(parent)
    , cameras_(cameras)
{
    setWindowTitle("Camera Calibration");
    setMinimumSize(700, 500);

    addPage(new CameraSelectionPage(cameras, this));
    addPage(new CaptureCalibrationPage(this));
    addPage(new ComputeCalibrationPage(this));
    addPage(new ReviewCalibrationPage(this));

    setStyleSheet(
        "QWizard { background-color: #2a2a30; }"
        "QLabel { color: #ccc; }"
        "QWizardPage { background-color: #2a2a30; }");
}

std::vector<CameraIntrinsics> CalibrationWizard::intrinsicsResults() const {
    return intrinsics_results_;
}

std::vector<CameraExtrinsics> CalibrationWizard::extrinsicsResults() const {
    return extrinsics_results_;
}

// --- CameraSelectionPage ---

CameraSelectionPage::CameraSelectionPage(
    const std::vector<std::shared_ptr<ICameraSource>>& cameras,
    QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Select Cameras");
    setSubTitle("Choose which cameras to include in calibration.");

    auto* layout = new QVBoxLayout(this);

    camera_list_ = new QListWidget(this);
    camera_list_->setStyleSheet(
        "QListWidget { background-color: #1e1e24; color: #ccc; border: 1px solid #444; }"
        "QListWidget::item:selected { background-color: #3a3a50; }");

    for (const auto& cam : cameras) {
        auto* item = new QListWidgetItem(
            QString::fromStdString(cam->displayName() + " (" + cam->id() + ")"));
        item->setCheckState(Qt::Checked);
        camera_list_->addItem(item);
    }

    layout->addWidget(new QLabel("Available cameras:", this));
    layout->addWidget(camera_list_);
    setLayout(layout);
}

bool CameraSelectionPage::isComplete() const {
    for (int i = 0; i < camera_list_->count(); ++i) {
        if (camera_list_->item(i)->checkState() == Qt::Checked) return true;
    }
    return false;
}

std::vector<int> CameraSelectionPage::selectedCameraIndices() const {
    std::vector<int> indices;
    for (int i = 0; i < camera_list_->count(); ++i) {
        if (camera_list_->item(i)->checkState() == Qt::Checked) {
            indices.push_back(i);
        }
    }
    return indices;
}

// --- CaptureCalibrationPage ---

CaptureCalibrationPage::CaptureCalibrationPage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Capture Calibration Frames");
    setSubTitle("Position the calibration board and capture frames from different angles.");

    auto* layout = new QVBoxLayout(this);

    // Board settings
    auto* board_group = new QGroupBox("Calibration Board", this);
    auto* board_layout = new QHBoxLayout(board_group);

    board_type_combo_ = new QComboBox(this);
    board_type_combo_->addItems({"Checkerboard", "ChArUco", "Circles Grid"});
    board_layout->addWidget(new QLabel("Type:", this));
    board_layout->addWidget(board_type_combo_);

    board_cols_spin_ = new QSpinBox(this);
    board_cols_spin_->setRange(3, 20);
    board_cols_spin_->setValue(9);
    board_layout->addWidget(new QLabel("Cols:", this));
    board_layout->addWidget(board_cols_spin_);

    board_rows_spin_ = new QSpinBox(this);
    board_rows_spin_->setRange(3, 20);
    board_rows_spin_->setValue(6);
    board_layout->addWidget(new QLabel("Rows:", this));
    board_layout->addWidget(board_rows_spin_);

    square_size_spin_ = new QDoubleSpinBox(this);
    square_size_spin_->setRange(0.001, 1.0);
    square_size_spin_->setValue(0.025);
    square_size_spin_->setDecimals(3);
    square_size_spin_->setSuffix(" m");
    board_layout->addWidget(new QLabel("Square:", this));
    board_layout->addWidget(square_size_spin_);

    board_group->setLayout(board_layout);
    layout->addWidget(board_group);

    // Preview
    preview_label_ = new QLabel(this);
    preview_label_->setMinimumSize(320, 240);
    preview_label_->setAlignment(Qt::AlignCenter);
    preview_label_->setStyleSheet("QLabel { background-color: #1a1a2e; border: 1px solid #333; }");
    preview_label_->setText("Camera preview will appear here");
    layout->addWidget(preview_label_);

    // Capture controls
    auto* control_row = new QHBoxLayout();
    capture_btn_ = new QPushButton("Capture Frame", this);
    capture_btn_->setStyleSheet(
        "QPushButton { background-color: #2a6e3f; color: white; padding: 8px 16px; "
        "border-radius: 4px; }"
        "QPushButton:hover { background-color: #3a8e4f; }");
    connect(capture_btn_, &QPushButton::clicked, this, &CaptureCalibrationPage::onCaptureFrame);
    control_row->addWidget(capture_btn_);

    frame_count_label_ = new QLabel(
        QString("Captured: 0 / %1").arg(target_frames_), this);
    control_row->addWidget(frame_count_label_);
    control_row->addStretch();
    layout->addLayout(control_row);

    setLayout(layout);
}

void CaptureCalibrationPage::initializePage() {
    captured_frames_ = 0;
    frame_count_label_->setText(QString("Captured: 0 / %1").arg(target_frames_));
}

void CaptureCalibrationPage::onCaptureFrame() {
    // In production: grab frame, detect corners, store if valid
    captured_frames_++;
    frame_count_label_->setText(
        QString("Captured: %1 / %2").arg(captured_frames_).arg(target_frames_));

    if (captured_frames_ >= target_frames_) {
        capture_btn_->setEnabled(false);
        capture_btn_->setText("Enough frames captured");
    }

    emit completeChanged();
}

// --- ComputeCalibrationPage ---

ComputeCalibrationPage::ComputeCalibrationPage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Compute Calibration");
    setSubTitle("Running camera calibration algorithms...");

    auto* layout = new QVBoxLayout(this);

    status_label_ = new QLabel("Ready to calibrate.", this);
    layout->addWidget(status_label_);

    progress_bar_ = new QProgressBar(this);
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(0);
    progress_bar_->setStyleSheet(
        "QProgressBar { background-color: #1e1e24; border: 1px solid #444; border-radius: 3px; }"
        "QProgressBar::chunk { background-color: #00cc66; }");
    layout->addWidget(progress_bar_);

    run_btn_ = new QPushButton("Run Calibration", this);
    run_btn_->setStyleSheet(
        "QPushButton { background-color: #2a6e3f; color: white; padding: 8px 16px; "
        "border-radius: 4px; }"
        "QPushButton:hover { background-color: #3a8e4f; }");
    connect(run_btn_, &QPushButton::clicked, this, &ComputeCalibrationPage::onRunCalibration);
    layout->addWidget(run_btn_);

    layout->addStretch();
    setLayout(layout);
}

void ComputeCalibrationPage::initializePage() {
    progress_bar_->setValue(0);
    status_label_->setText("Ready to calibrate.");
    run_btn_->setEnabled(true);
}

void ComputeCalibrationPage::onRunCalibration() {
    run_btn_->setEnabled(false);

    // Step 1: Intrinsics
    status_label_->setText("Computing intrinsics (per-camera)...");
    progress_bar_->setValue(25);
    // In production: cv::calibrateCamera for each camera

    // Step 2: Extrinsics
    status_label_->setText("Computing extrinsics (multi-camera)...");
    progress_bar_->setValue(60);
    // In production: cv::stereoCalibrate or bundle adjustment

    // Step 3: Done
    status_label_->setText("Calibration complete!");
    progress_bar_->setValue(100);

    emit completeChanged();
}

// --- ReviewCalibrationPage ---

ReviewCalibrationPage::ReviewCalibrationPage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Review Results");
    setSubTitle("Review calibration accuracy before accepting.");

    auto* layout = new QVBoxLayout(this);

    summary_label_ = new QLabel(this);
    layout->addWidget(summary_label_);

    results_table_ = new QTableWidget(this);
    results_table_->setColumnCount(4);
    results_table_->setHorizontalHeaderLabels(
        {"Camera", "Reproj. Error (px)", "Fx", "Fy"});
    results_table_->setStyleSheet(
        "QTableWidget { background-color: #1e1e24; color: #ccc; border: 1px solid #444; "
        "gridline-color: #333; }"
        "QHeaderView::section { background-color: #2a2a30; color: #aaa; "
        "border: 1px solid #333; padding: 3px; }");
    results_table_->horizontalHeader()->setStretchLastSection(true);
    layout->addWidget(results_table_);

    setLayout(layout);
}

void ReviewCalibrationPage::initializePage() {
    // In production: populate with actual results from CalibrationWizard
    summary_label_->setText(
        "Calibration results will be displayed here.\n"
        "Accept to save calibration files, or go back to recapture.");

    results_table_->setRowCount(0);
}

}  // namespace mocap
