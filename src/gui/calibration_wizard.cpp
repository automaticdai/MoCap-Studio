#include "gui/calibration_wizard.h"
#include "capture/usb_camera_source.h"
#include "capture/ip_camera_source.h"
#include "capture/video_file_camera_source.h"
#include "core/types.h"
#include <QCheckBox>
#include <QCoreApplication>
#include <QEventLoop>
#include <QHeaderView>
#include <QLayoutItem>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QPixmap>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <future>
#include <cmath>
#include <limits>

namespace mocap {

namespace {

// Construct + open an ICameraSource matching the config's type. Returns nullptr
// on unknown type or failed open. Mirrors `MainWindow::initializeCamerasFromConfig`
// so the wizard behaves the same as the live pipeline.
std::shared_ptr<ICameraSource> makeOpenedSource(const CameraConfig& cfg) {
    std::shared_ptr<ICameraSource> src;
    if (cfg.type == "usb") {
        src = std::make_shared<UsbCameraSource>();
    } else if (cfg.type == "ip") {
        src = std::make_shared<IpCameraSource>();
    } else if (cfg.type == "file" || cfg.type == "video") {
        src = std::make_shared<VideoFileCameraSource>();
    } else {
        spdlog::warn("CalibrationWizard: unknown camera type '{}'", cfg.type);
        return nullptr;
    }
    if (!src->open(cfg)) {
        spdlog::warn("CalibrationWizard: failed to open camera '{}'", cfg.id);
        return nullptr;
    }
    return src;
}

// Convert a BGR cv::Mat to a QImage that owns its pixels (deep copy).
QImage matToQImage(const cv::Mat& bgr) {
    if (bgr.empty()) return {};
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    QImage img(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
               QImage::Format_RGB888);
    return img.copy();
}

}  // namespace

// =====================================================================
// ReprojErrorHistogram
// =====================================================================

ReprojErrorHistogram::ReprojErrorHistogram(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(sizeHint());
    setStyleSheet("background-color: #1e1e24;");
}

void ReprojErrorHistogram::setData(const std::vector<double>& errors_px, double threshold_px) {
    errors_ = errors_px;
    threshold_ = threshold_px;
    update();
}

void ReprojErrorHistogram::paintEvent(QPaintEvent* /*event*/) {
    QPainter p(this);
    p.fillRect(rect(), QColor(0x1e, 0x1e, 0x24));

    if (errors_.empty()) {
        p.setPen(QColor(0x88, 0x88, 0x88));
        p.drawText(rect(), Qt::AlignCenter, "no data");
        return;
    }

    const double y_max = std::max(threshold_ * 1.5,
                                  *std::max_element(errors_.begin(), errors_.end()));
    const int w = width();
    const int h = height();
    const int n = static_cast<int>(errors_.size());
    const double bar_w = static_cast<double>(w) / n;

    // Bars: green if under threshold, amber over.
    for (int i = 0; i < n; ++i) {
        const double e = errors_[i];
        const int bar_h = static_cast<int>((e / y_max) * (h - 8));
        const int x = static_cast<int>(i * bar_w);
        const int bw = std::max(1, static_cast<int>(bar_w) - 1);
        QColor col = e <= threshold_ ? QColor(0x4a, 0xc0, 0x6e)
                                     : QColor(0xe0, 0x8a, 0x2a);
        p.fillRect(x, h - bar_h - 1, bw, bar_h, col);
    }

    // Threshold line.
    const int y_thresh = h - 1 - static_cast<int>((threshold_ / y_max) * (h - 8));
    p.setPen(QPen(QColor(0xcc, 0x44, 0x44), 1, Qt::DashLine));
    p.drawLine(0, y_thresh, w, y_thresh);
}

// =====================================================================
// CalibrationWizard
// =====================================================================

CalibrationWizard::CalibrationWizard(
    const std::vector<CameraConfig>& camera_configs,
    QWidget* parent)
    : QWizard(parent)
    , camera_configs_(camera_configs)
{
    setWindowTitle("Camera Calibration");
    setMinimumSize(800, 600);

    sources_.reserve(camera_configs_.size());
    for (const auto& cfg : camera_configs_) {
        sources_.push_back(makeOpenedSource(cfg));
    }

    addPage(new CameraSelectionPage(this));
    addPage(new CaptureCalibrationPage(this));
    addPage(new ComputeCalibrationPage(this));
    addPage(new ReviewCalibrationPage(this));

    setStyleSheet(
        "QWizard { background-color: #2a2a30; }"
        "QLabel { color: #ccc; }"
        "QWizardPage { background-color: #2a2a30; }"
        "QSpinBox, QDoubleSpinBox, QComboBox { background-color: #1e1e24; color: #ccc; "
        "border: 1px solid #444; padding: 2px; }");
}

CalibrationWizard::~CalibrationWizard() {
    for (auto& src : sources_) {
        if (src) src->close();
    }
}

ICameraSource* CalibrationWizard::cameraSource(int config_index) {
    if (config_index < 0 || config_index >= static_cast<int>(sources_.size())) return nullptr;
    return sources_[config_index].get();
}

void CalibrationWizard::setSelectedCameraIndices(const std::vector<int>& indices) {
    selected_indices_ = indices;
    clearCaptures();
}

std::vector<std::vector<cv::Point2f>>& CalibrationWizard::cornersForSlot(int slot) {
    return per_slot_corners_.at(slot);
}

cv::Size& CalibrationWizard::imageSizeForSlot(int slot) {
    return per_slot_image_size_.at(slot);
}

void CalibrationWizard::setResultForSlot(int slot, const IntrinsicsResult& result) {
    per_slot_results_.at(slot) = result;
}

const IntrinsicsResult& CalibrationWizard::resultForSlot(int slot) const {
    return per_slot_results_.at(slot);
}

std::vector<CameraIntrinsics> CalibrationWizard::intrinsicsResults() const {
    std::vector<CameraIntrinsics> out;
    out.reserve(per_slot_results_.size());
    for (const auto& r : per_slot_results_) {
        out.push_back(r.intrinsics);
    }
    return out;
}

std::vector<CameraExtrinsics> CalibrationWizard::extrinsicsResults() const {
    return extrinsics_;
}

void CalibrationWizard::clearCaptures() {
    per_slot_corners_.assign(selected_indices_.size(), {});
    per_slot_image_size_.assign(selected_indices_.size(), cv::Size());
    per_slot_results_.assign(selected_indices_.size(), IntrinsicsResult{});
    stereo_results_.clear();
    extrinsics_.clear();
}

bool CalibrationWizard::addPairedCapture(
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<cv::Size>& sizes) {
    if (corners.empty() || corners.size() != selected_indices_.size() || sizes.size() != corners.size())
        return false;
    for (size_t i = 0; i < corners.size(); ++i) {
        if (corners[i].size() != static_cast<size_t>(board_.cols * board_.rows) ||
            sizes[i].width <= 0 || sizes[i].height <= 0 ||
            (per_slot_image_size_[i].area() && per_slot_image_size_[i] != sizes[i])) return false;
        for (const auto& point : corners[i])
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
    }
    for (size_t i = 0; i < corners.size(); ++i) {
        per_slot_corners_[i].push_back(corners[i]);
        per_slot_image_size_[i] = sizes[i];
    }
    per_slot_results_.assign(selected_indices_.size(), IntrinsicsResult{});
    stereo_results_.clear();
    extrinsics_.clear();
    return true;
}

bool CalibrationWizard::computeExtrinsics(QString& error) {
    error.clear();
    extrinsics_.clear();
    stereo_results_.clear();
    for (const auto& result : per_slot_results_) {
        if (!result.success || !std::isfinite(result.rms) || result.rms > 2.0) {
            error = "Lens calibration failed or exceeds 2 px RMS. Recapture varied board views.";
            return false;
        }
    }
    if (numSlots() < 2) return numSlots() == 1; // Single-camera lens calibration only.
    std::vector<CameraExtrinsics> extrinsics(numSlots());
    // The first camera defines a right-handed, Y-up world frame: X right,
    // Y up, Z behind the camera. OpenCV camera coordinates are Y down/Z forward.
    extrinsics[0].rotation = (cv::Mat_<double>(3, 3) << 1,0,0, 0,-1,0, 0,0,-1);
    for (int slot = 1; slot < numSlots(); ++slot) {
        const auto result = calibrateStereo(per_slot_corners_[0], per_slot_corners_[slot],
            per_slot_results_[0].intrinsics, per_slot_results_[slot].intrinsics, board_);
        stereo_results_.push_back(result);
        if (!result.success) {
            error = QString::fromStdString(camera_configs_[selected_indices_[slot]].id + ": " + result.error);
            return false;
        }
        extrinsics[slot] = result.extrinsics;
        extrinsics[slot].rotation = result.extrinsics.rotation * extrinsics[0].rotation;
    }
    extrinsics_ = std::move(extrinsics);
    return true;
}

// =====================================================================
// CameraSelectionPage
// =====================================================================

CameraSelectionPage::CameraSelectionPage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Select Cameras");
    setSubTitle("Select both cameras for 3D calibration, or one camera for lens calibration only.");

    auto* layout = new QVBoxLayout(this);

    camera_list_ = new QListWidget(this);
    camera_list_->setStyleSheet(
        "QListWidget { background-color: #1e1e24; color: #ccc; border: 1px solid #444; }"
        "QListWidget::item:selected { background-color: #3a3a50; }");
    layout->addWidget(new QLabel("Available cameras:", this));
    layout->addWidget(camera_list_);
    setLayout(layout);

    connect(camera_list_, &QListWidget::itemChanged,
            this, [this]() { emit completeChanged(); });
}

void CameraSelectionPage::initializePage() {
    camera_list_->clear();
    auto* w = static_cast<CalibrationWizard*>(wizard());
    const auto& configs = w->cameraConfigs();
    for (size_t i = 0; i < configs.size(); ++i) {
        const auto& cfg = configs[i];
        QString text = QString::fromStdString(cfg.id + " (" + cfg.type + ")");
        if (!w->cameraSource(static_cast<int>(i))) {
            text += "  [unavailable]";
        }
        auto* item = new QListWidgetItem(text);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(w->cameraSource(static_cast<int>(i)) ? Qt::Checked : Qt::Unchecked);
        if (!w->cameraSource(static_cast<int>(i))) {
            item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
        }
        camera_list_->addItem(item);
    }
}

bool CameraSelectionPage::isComplete() const {
    for (int i = 0; i < camera_list_->count(); ++i) {
        if (camera_list_->item(i)->checkState() == Qt::Checked) return true;
    }
    return false;
}

bool CameraSelectionPage::validatePage() {
    std::vector<int> indices;
    for (int i = 0; i < camera_list_->count(); ++i) {
        if (camera_list_->item(i)->checkState() == Qt::Checked) {
            indices.push_back(i);
        }
    }
    if (indices.empty()) return false;
    static_cast<CalibrationWizard*>(wizard())->setSelectedCameraIndices(indices);
    return true;
}

// =====================================================================
// CaptureCalibrationPage
// =====================================================================

CaptureCalibrationPage::CaptureCalibrationPage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Capture Calibration Frames");
    setSubTitle("Keep the cameras fixed. Hold the same board still in every camera before Capture. "
                "Collect at least 8 shared views at varied tilts and distances; do not flip the board.");

    auto* layout = new QVBoxLayout(this);

    // Board settings.
    auto* board_group = new QGroupBox("Calibration Board", this);
    auto* board_layout = new QHBoxLayout(board_group);

    board_type_combo_ = new QComboBox(this);
    board_type_combo_->addItem("Checkerboard");
    // ChArUco and circle-grid are out of scope for Phase 1; surface the
    // combo for forward-compat but only the default is wired up.
    board_layout->addWidget(new QLabel("Type:", this));
    board_layout->addWidget(board_type_combo_);

    board_cols_spin_ = new QSpinBox(this);
    board_cols_spin_->setRange(3, 20);
    board_cols_spin_->setValue(9);
    board_layout->addWidget(new QLabel("Inner cols:", this));
    board_layout->addWidget(board_cols_spin_);

    board_rows_spin_ = new QSpinBox(this);
    board_rows_spin_->setRange(3, 20);
    board_rows_spin_->setValue(6);
    board_layout->addWidget(new QLabel("Inner rows:", this));
    board_layout->addWidget(board_rows_spin_);

    square_size_spin_ = new QDoubleSpinBox(this);
    square_size_spin_->setRange(0.001, 1.0);
    square_size_spin_->setDecimals(3);
    square_size_spin_->setValue(0.025);
    square_size_spin_->setSuffix(" m");
    board_layout->addWidget(new QLabel("Square:", this));
    board_layout->addWidget(square_size_spin_);

    board_group->setLayout(board_layout);
    layout->addWidget(board_group);

    connect(board_cols_spin_, qOverload<int>(&QSpinBox::valueChanged),
            this, &CaptureCalibrationPage::onBoardParamChanged);
    connect(board_rows_spin_, qOverload<int>(&QSpinBox::valueChanged),
            this, &CaptureCalibrationPage::onBoardParamChanged);
    connect(square_size_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &CaptureCalibrationPage::onBoardParamChanged);

    // Preview of the most recent captured frame (with corner overlay if detected).
    preview_label_ = new QLabel(this);
    preview_label_->setMinimumSize(480, 320);
    preview_label_->setAlignment(Qt::AlignCenter);
    preview_label_->setStyleSheet(
        "QLabel { background-color: #1a1a2e; border: 1px solid #333; color: #888; }");
    preview_label_->setText("Hold the board still in all cameras, then press Capture Frame.");
    layout->addWidget(preview_label_);

    // Per-camera capture counts.
    count_list_ = new QListWidget(this);
    count_list_->setStyleSheet(
        "QListWidget { background-color: #1e1e24; color: #ccc; border: 1px solid #444; }");
    count_list_->setMaximumHeight(120);
    layout->addWidget(count_list_);

    // Controls.
    auto* control_row = new QHBoxLayout();
    capture_btn_ = new QPushButton("Capture Frame", this);
    capture_btn_->setStyleSheet(
        "QPushButton { background-color: #2a6e3f; color: white; padding: 8px 16px; "
        "border-radius: 4px; }"
        "QPushButton:hover { background-color: #3a8e4f; }"
        "QPushButton:disabled { background-color: #444; color: #888; }");
    connect(capture_btn_, &QPushButton::clicked,
            this, &CaptureCalibrationPage::onCaptureFrame);
    control_row->addWidget(capture_btn_);

    status_label_ = new QLabel(this);
    status_label_->setWordWrap(true);
    control_row->addWidget(status_label_);
    control_row->addStretch();
    layout->addLayout(control_row);

    setLayout(layout);
}

void CaptureCalibrationPage::initializePage() {
    auto* w = static_cast<CalibrationWizard*>(wizard());

    // Sync UI defaults into the shared board spec.
    refreshBoardSpecFromUi();

    count_list_->clear();
    const auto& sel = w->selectedCameraIndices();
    for (int idx : sel) {
        const auto& cfg = w->cameraConfigs()[idx];
        count_list_->addItem(QString::fromStdString(cfg.id + ": 0 detected"));
    }
    refreshCounts();
    status_label_->setText("");
    preview_label_->setText("Hold the board still in all cameras, then press Capture Frame.");
    preview_label_->setPixmap({});
    capture_btn_->setEnabled(true);
}

bool CaptureCalibrationPage::isComplete() const {
    auto* w = static_cast<const CalibrationWizard*>(wizard());
    if (!w) return false;
    const int n_slots = w->numSlots();
    if (n_slots == 0) return false;
    for (int s = 0; s < n_slots; ++s) {
        const auto& corners = const_cast<CalibrationWizard*>(w)->cornersForSlot(s);
        if (static_cast<int>(corners.size()) < min_frames_required_) return false;
    }
    return true;
}

void CaptureCalibrationPage::refreshBoardSpecFromUi() {
    auto* w = static_cast<CalibrationWizard*>(wizard());
    w->boardSpec().cols = board_cols_spin_->value();
    w->boardSpec().rows = board_rows_spin_->value();
    w->boardSpec().square_size_m = square_size_spin_->value();
}

void CaptureCalibrationPage::onBoardParamChanged() {
    if (!wizard()) return;
    refreshBoardSpecFromUi();
    static_cast<CalibrationWizard*>(wizard())->clearCaptures();
    refreshCounts();
    status_label_->setText("Board settings changed — captures cleared.");
    emit completeChanged();
}

void CaptureCalibrationPage::onCaptureFrame() {
    auto* w = static_cast<CalibrationWizard*>(wizard());
    refreshBoardSpecFromUi();
    const auto& spec = w->boardSpec();
    const auto& sel = w->selectedCameraIndices();

    capture_btn_->setEnabled(false);
    // Grab all streams before expensive corner detection. Concurrent grabs avoid
    // adding one camera's blocking read time to the next camera's observation.
    std::vector<std::future<CapturedFrame>> pending;
    for (int index : sel) {
        auto* source = w->cameraSource(index);
        pending.push_back(std::async(std::launch::async, [source] {
            CapturedFrame frame;
            try {
                if (source) source->grabFrame(frame, 500);
            } catch (const cv::Exception& e) {
                spdlog::warn("Calibration capture failed: {}", e.what());
                frame = {};
            }
            return frame;
        }));
    }
    std::vector<CapturedFrame> frames;
    for (auto& future : pending) frames.push_back(future.get());
    std::vector<std::vector<cv::Point2f>> corners(sel.size());
    std::vector<cv::Size> sizes(sel.size());
    std::vector<cv::Mat> previews;
    int found_count = 0;
    bool fresh = true;
    double min_ts = std::numeric_limits<double>::infinity(), max_ts = -min_ts;
    const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    for (size_t slot = 0; slot < frames.size(); ++slot) {
        auto& frame = frames[slot];
        if (frame.image.empty()) { fresh = false; continue; }
        const auto* source = w->cameraSource(sel[slot]);
        if (source->isLive()) {
            fresh = fresh && std::isfinite(frame.timestamp) && now - frame.timestamp <= 0.5;
            min_ts = std::min(min_ts, frame.timestamp);
            max_ts = std::max(max_ts, frame.timestamp);
        }
        const bool found = detectCheckerboard(frame.image, spec, corners[slot]);
        if (found) ++found_count;
        sizes[slot] = frame.image.size();
        cv::Mat preview = frame.image.clone();
        if (found) cv::drawChessboardCorners(preview, {spec.cols, spec.rows}, corners[slot], true);
        cv::resize(preview, preview, cv::Size(320, 180));
        cv::putText(preview, w->cameraConfigs()[sel[slot]].id, {8, 20},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
        previews.push_back(preview);
    }
    if (!previews.empty()) {
        cv::Mat combined;
        cv::hconcat(previews, combined);
        preview_label_->setText({});
        preview_label_->setPixmap(QPixmap::fromImage(matToQImage(combined)).scaled(
            preview_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
    // This is a stationary-board workflow, not hardware synchronization.
    // Reject stale or widely separated observations rather than mismatching poses.
    fresh = fresh && (max_ts - min_ts <= 0.1);
    const bool saved = fresh && found_count == static_cast<int>(sel.size()) &&
                       w->addPairedCapture(corners, sizes);
    refreshCounts();
    if (saved) {
        status_label_->setText("Paired view saved. Move and tilt the board for the next view.");
        status_label_->setStyleSheet("QLabel { color: #4ac06e; }");
    } else {
        status_label_->setText(!fresh
            ? "Frames missing, stale or over 100 ms apart — hold the board still and retry."
            : QString("Not saved: board detected in %1 / %2 cameras. Check visibility and resolution.")
                  .arg(found_count).arg(sel.size()));
        status_label_->setStyleSheet("QLabel { color: #e08a2a; }");
    }
    capture_btn_->setEnabled(true);

    emit completeChanged();
}

void CaptureCalibrationPage::refreshCounts() {
    auto* w = static_cast<CalibrationWizard*>(wizard());
    const auto& sel = w->selectedCameraIndices();
    for (int s = 0; s < static_cast<int>(sel.size()); ++s) {
        const int count = static_cast<int>(w->cornersForSlot(s).size());
        const auto& cfg = w->cameraConfigs()[sel[s]];
        QString text = QString::fromStdString(cfg.id) +
                       QString(": %1 paired views").arg(count);
        if (count >= min_frames_required_) text += "  ✓";
        if (auto* item = count_list_->item(s)) item->setText(text);
    }
}

// =====================================================================
// ComputeCalibrationPage
// =====================================================================

ComputeCalibrationPage::ComputeCalibrationPage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Compute Calibration");
    setSubTitle("Compute lens calibration and relative camera positions from the paired views.");

    auto* layout = new QVBoxLayout(this);

    status_label_ = new QLabel("Ready to calibrate.", this);
    status_label_->setWordWrap(true);
    layout->addWidget(status_label_);

    progress_bar_ = new QProgressBar(this);
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(0);
    progress_bar_->setStyleSheet(
        "QProgressBar { background-color: #1e1e24; border: 1px solid #444; "
        "border-radius: 3px; color: #ccc; text-align: center; }"
        "QProgressBar::chunk { background-color: #4ac06e; }");
    layout->addWidget(progress_bar_);

    run_btn_ = new QPushButton("Run Calibration", this);
    run_btn_->setStyleSheet(
        "QPushButton { background-color: #2a6e3f; color: white; padding: 8px 16px; "
        "border-radius: 4px; }"
        "QPushButton:hover { background-color: #3a8e4f; }"
        "QPushButton:disabled { background-color: #444; color: #888; }");
    connect(run_btn_, &QPushButton::clicked,
            this, &ComputeCalibrationPage::onRunCalibration);
    layout->addWidget(run_btn_);

    layout->addStretch();
    setLayout(layout);
}

void ComputeCalibrationPage::initializePage() {
    progress_bar_->setValue(0);
    status_label_->setText("Ready to calibrate.");
    run_btn_->setEnabled(true);
    calibration_done_ = false;
}

bool ComputeCalibrationPage::isComplete() const {
    return calibration_done_;
}

void ComputeCalibrationPage::onRunCalibration() {
    auto* w = static_cast<CalibrationWizard*>(wizard());
    run_btn_->setEnabled(false);

    const int n_slots = w->numSlots();
    if (n_slots == 0) {
        status_label_->setText("No cameras selected.");
        run_btn_->setEnabled(true);
        return;
    }

    int successes = 0;
    for (int s = 0; s < n_slots; ++s) {
        const int cam_idx = w->selectedCameraIndices()[s];
        const auto& cfg = w->cameraConfigs()[cam_idx];
        status_label_->setText(QString("Calibrating %1...").arg(QString::fromStdString(cfg.id)));
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

        const auto& corners = w->cornersForSlot(s);
        const cv::Size image_size = w->imageSizeForSlot(s);
        const auto result = calibrateIntrinsics(corners, image_size, w->boardSpec());
        w->setResultForSlot(s, result);
        if (result.success) ++successes;

        progress_bar_->setValue(static_cast<int>(100.0 * (s + 1) / n_slots));
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
    }

    QString stereo_error;
    calibration_done_ = successes == n_slots && w->computeExtrinsics(stereo_error);
    if (calibration_done_) {
        status_label_->setText(QString("Calibration complete (%1 cameras).").arg(n_slots));
    } else {
        status_label_->setText(
            QString("Calibration finished with errors: %1 / %2 cameras succeeded.")
                .arg(successes).arg(n_slots));
    }
    progress_bar_->setValue(100);
    if (!stereo_error.isEmpty()) status_label_->setText(stereo_error);
    run_btn_->setEnabled(!calibration_done_);
    emit completeChanged();
}

// =====================================================================
// ReviewCalibrationPage
// =====================================================================

ReviewCalibrationPage::ReviewCalibrationPage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Review Results");
    setSubTitle("Review lens error, stereo error and camera separation before saving.");

    auto* layout = new QVBoxLayout(this);

    summary_label_ = new QLabel(this);
    summary_label_->setWordWrap(true);
    layout->addWidget(summary_label_);

    results_table_ = new QTableWidget(this);
    results_table_->setColumnCount(5);
    results_table_->setHorizontalHeaderLabels(
        {"Camera", "RMS (px)", "fx", "fy", "principal pt"});
    results_table_->setStyleSheet(
        "QTableWidget { background-color: #1e1e24; color: #ccc; border: 1px solid #444; "
        "gridline-color: #333; }"
        "QHeaderView::section { background-color: #2a2a30; color: #aaa; "
        "border: 1px solid #333; padding: 3px; }");
    results_table_->horizontalHeader()->setStretchLastSection(true);
    results_table_->verticalHeader()->setVisible(false);
    layout->addWidget(results_table_);

    layout->addWidget(new QLabel("Per-frame reprojection error (px):", this));

    auto* histograms_wrapper = new QWidget(this);
    histograms_layout_ = new QVBoxLayout(histograms_wrapper);
    histograms_layout_->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(histograms_wrapper);

    setLayout(layout);
}

void ReviewCalibrationPage::initializePage() {
    auto* w = static_cast<CalibrationWizard*>(wizard());
    const int n_slots = w->numSlots();

    results_table_->setRowCount(n_slots);

    // Clear any histograms from a previous run.
    while (QLayoutItem* item = histograms_layout_->takeAt(0)) {
        if (auto* widget = item->widget()) widget->deleteLater();
        delete item;
    }

    int success_count = 0;
    double worst_rms = 0.0;

    for (int s = 0; s < n_slots; ++s) {
        const int cam_idx = w->selectedCameraIndices()[s];
        const auto& cfg = w->cameraConfigs()[cam_idx];
        const auto& r = w->resultForSlot(s);

        auto add_cell = [&](int col, const QString& text) {
            auto* item = new QTableWidgetItem(text);
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            results_table_->setItem(s, col, item);
        };

        add_cell(0, QString::fromStdString(cfg.id));
        if (r.success) {
            add_cell(1, QString::number(r.rms, 'f', 3));
            add_cell(2, QString::number(r.intrinsics.fx, 'f', 1));
            add_cell(3, QString::number(r.intrinsics.fy, 'f', 1));
            add_cell(4, QString("(%1, %2)")
                            .arg(r.intrinsics.cx, 0, 'f', 1)
                            .arg(r.intrinsics.cy, 0, 'f', 1));
            ++success_count;
            worst_rms = std::max(worst_rms, r.rms);
        } else {
            add_cell(1, "FAILED");
            add_cell(2, "—");
            add_cell(3, "—");
            add_cell(4, "—");
        }

        // Per-camera histogram row: label + histogram side by side.
        auto* row = new QWidget();
        auto* row_layout = new QHBoxLayout(row);
        row_layout->setContentsMargins(0, 2, 0, 2);
        auto* lbl = new QLabel(QString::fromStdString(cfg.id));
        lbl->setMinimumWidth(80);
        row_layout->addWidget(lbl);
        auto* hist = new ReprojErrorHistogram();
        hist->setData(r.per_view_errors, /*threshold_px=*/1.0);
        row_layout->addWidget(hist, 1);
        histograms_layout_->addWidget(row);
    }

    QString summary;
    if (success_count == n_slots && n_slots > 0) {
        const QString verdict = worst_rms < 1.0
            ? "<b style='color:#4ac06e'>Excellent</b>"
            : (worst_rms < 2.0 ? "<b style='color:#e0c060'>Acceptable</b>"
                               : "<b style='color:#cc4444'>Poor</b>");
        summary = QString("%1 camera(s) calibrated. Worst RMS: %2 px. %3<br>"
                          "Click Finish to save results, or Back to recapture.")
                      .arg(n_slots).arg(worst_rms, 0, 'f', 3).arg(verdict);
    } else {
        summary = QString("<b style='color:#cc4444'>%1 / %2 cameras failed.</b> "
                          "Use Back to recapture more frames.")
                      .arg(n_slots - success_count).arg(n_slots);
    }
    if (!w->stereoResults().empty()) {
        summary += "<br>World origin: first camera; X right, Y up, Z behind the camera. Units: metres.";
        for (size_t i = 0; i < w->stereoResults().size(); ++i) {
            const auto& result = w->stereoResults()[i];
            summary += QString("<br>%1: stereo RMS %2 px; separation %3 m.")
                .arg(QString::fromStdString(w->cameraConfigs()[w->selectedCameraIndices()[i + 1]].id).toHtmlEscaped())
                .arg(result.rms, 0, 'f', 3).arg(cv::norm(result.extrinsics.translation), 0, 'f', 3);
        }
    } else {
        summary += "<br>Lens calibration only. Select at least two cameras to enable 3D.";
    }
    summary_label_->setText(summary);
}

}  // namespace mocap
