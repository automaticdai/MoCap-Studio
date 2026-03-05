#include "gui/settings_dialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QLabel>
#include <algorithm>

namespace mocap {

SettingsDialog::SettingsDialog(const AppConfig& config, QWidget* parent)
    : QDialog(parent), config_(config)
{
    setWindowTitle("Settings");
    setMinimumSize(520, 480);
    setupUi();
}

void SettingsDialog::setupUi() {
    auto* layout = new QVBoxLayout(this);

    auto* tabs = new QTabWidget;
    tabs->addTab(createCaptureTab(), "Capture");
    tabs->addTab(createPoseEstimationTab(), "Pose Estimation");
    tabs->addTab(createTriangulationTab(), "Triangulation");
    tabs->addTab(createSkeletonTab(), "Skeleton");
    tabs->addTab(createGuiTab(), "GUI");
    layout->addWidget(tabs);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);

    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

QWidget* SettingsDialog::createCaptureTab() {
    auto* widget = new QWidget;
    auto* form = new QFormLayout(widget);

    capture_fps_spin_ = new QSpinBox;
    capture_fps_spin_->setRange(1, 240);
    capture_fps_spin_->setValue(config_.capture.target_fps);
    capture_fps_spin_->setSuffix(" fps");
    form->addRow("Target FPS:", capture_fps_spin_);

    sync_mode_combo_ = new QComboBox;
    sync_mode_combo_->addItems({"software", "hardware", "genlock"});
    sync_mode_combo_->setCurrentText(
        QString::fromStdString(config_.capture.sync_mode));
    form->addRow("Sync Mode:", sync_mode_combo_);

    max_skew_spin_ = new QDoubleSpinBox;
    max_skew_spin_->setRange(0.1, 100.0);
    max_skew_spin_->setDecimals(1);
    max_skew_spin_->setSuffix(" ms");
    max_skew_spin_->setValue(config_.capture.max_sync_skew_ms);
    form->addRow("Max Sync Skew:", max_skew_spin_);

    return widget;
}

QWidget* SettingsDialog::createPoseEstimationTab() {
    auto* widget = new QWidget;
    auto* form = new QFormLayout(widget);

    pe_backend_combo_ = new QComboBox;
    pe_backend_combo_->addItems({"onnxruntime"});
    pe_backend_combo_->setCurrentText(
        QString::fromStdString(config_.pose_estimation.backend));
    form->addRow("Backend:", pe_backend_combo_);

    auto* modelRow = new QHBoxLayout;
    pe_model_edit_ = new QLineEdit(
        QString::fromStdString(config_.pose_estimation.model));
    auto* browseBtn = new QPushButton("Browse...");
    modelRow->addWidget(pe_model_edit_);
    modelRow->addWidget(browseBtn);
    form->addRow("Model:", modelRow);

    connect(browseBtn, &QPushButton::clicked, this, [this]() {
        QString path = QFileDialog::getOpenFileName(
            this, "Select Model File", {},
            "ONNX Models (*.onnx);;All Files (*)");
        if (!path.isEmpty()) {
            pe_model_edit_->setText(path);
        }
    });

    pe_device_combo_ = new QComboBox;
    pe_device_combo_->addItems({"cuda:0", "cuda:1", "cpu"});
    pe_device_combo_->setCurrentText(
        QString::fromStdString(config_.pose_estimation.device));
    pe_device_combo_->setEditable(true);
    form->addRow("Device:", pe_device_combo_);

    pe_detect_thresh_spin_ = new QDoubleSpinBox;
    pe_detect_thresh_spin_->setRange(0.0, 1.0);
    pe_detect_thresh_spin_->setDecimals(2);
    pe_detect_thresh_spin_->setSingleStep(0.05);
    pe_detect_thresh_spin_->setValue(config_.pose_estimation.detection_threshold);
    form->addRow("Detection Threshold:", pe_detect_thresh_spin_);

    pe_keypoint_thresh_spin_ = new QDoubleSpinBox;
    pe_keypoint_thresh_spin_->setRange(0.0, 1.0);
    pe_keypoint_thresh_spin_->setDecimals(2);
    pe_keypoint_thresh_spin_->setSingleStep(0.05);
    pe_keypoint_thresh_spin_->setValue(config_.pose_estimation.keypoint_threshold);
    form->addRow("Keypoint Threshold:", pe_keypoint_thresh_spin_);

    return widget;
}

QWidget* SettingsDialog::createTriangulationTab() {
    auto* widget = new QWidget;
    auto* form = new QFormLayout(widget);

    tri_min_views_spin_ = new QSpinBox;
    tri_min_views_spin_->setRange(2, 16);
    tri_min_views_spin_->setValue(config_.triangulation.min_views);
    form->addRow("Min Views:", tri_min_views_spin_);

    tri_ransac_check_ = new QCheckBox("Enable RANSAC");
    tri_ransac_check_->setChecked(config_.triangulation.ransac_enabled);
    form->addRow(tri_ransac_check_);

    tri_ransac_thresh_spin_ = new QDoubleSpinBox;
    tri_ransac_thresh_spin_->setRange(0.1, 100.0);
    tri_ransac_thresh_spin_->setDecimals(1);
    tri_ransac_thresh_spin_->setSuffix(" px");
    tri_ransac_thresh_spin_->setValue(config_.triangulation.ransac_threshold_px);
    form->addRow("RANSAC Threshold:", tri_ransac_thresh_spin_);

    // Enable/disable RANSAC threshold based on checkbox
    connect(tri_ransac_check_, &QCheckBox::toggled,
            tri_ransac_thresh_spin_, &QWidget::setEnabled);
    tri_ransac_thresh_spin_->setEnabled(config_.triangulation.ransac_enabled);

    tri_filter_combo_ = new QComboBox;
    tri_filter_combo_->addItems({"butterworth", "savitzky_golay", "none"});
    tri_filter_combo_->setCurrentText(
        QString::fromStdString(config_.triangulation.temporal_filter));
    form->addRow("Temporal Filter:", tri_filter_combo_);

    tri_cutoff_spin_ = new QDoubleSpinBox;
    tri_cutoff_spin_->setRange(0.1, 30.0);
    tri_cutoff_spin_->setDecimals(1);
    tri_cutoff_spin_->setSuffix(" Hz");
    tri_cutoff_spin_->setValue(config_.triangulation.filter_cutoff_hz);
    form->addRow("Filter Cutoff:", tri_cutoff_spin_);

    return widget;
}

QWidget* SettingsDialog::createSkeletonTab() {
    auto* widget = new QWidget;
    auto* form = new QFormLayout(widget);

    skel_def_combo_ = new QComboBox;
    skel_def_combo_->addItems({"body_25", "coco_17", "halpe_26"});
    skel_def_combo_->setCurrentText(
        QString::fromStdString(config_.skeleton.definition));
    form->addRow("Skeleton Definition:", skel_def_combo_);

    skel_solver_combo_ = new QComboBox;
    skel_solver_combo_->addItems({"analytical", "optimisation"});
    skel_solver_combo_->setCurrentText(
        QString::fromStdString(config_.skeleton.ik_solver));
    form->addRow("IK Solver:", skel_solver_combo_);

    skel_limits_check_ = new QCheckBox("Enable Joint Limits");
    skel_limits_check_->setChecked(config_.skeleton.joint_limits_enabled);
    form->addRow(skel_limits_check_);

    return widget;
}

QWidget* SettingsDialog::createGuiTab() {
    auto* widget = new QWidget;
    auto* layout = new QVBoxLayout(widget);

    auto* form = new QFormLayout;

    gui_fps_spin_ = new QSpinBox;
    gui_fps_spin_->setRange(15, 144);
    gui_fps_spin_->setValue(config_.gui.canvas_fps);
    gui_fps_spin_->setSuffix(" fps");
    form->addRow("Canvas FPS:", gui_fps_spin_);

    gui_palette_combo_ = new QComboBox;
    gui_palette_combo_->addItems({"oklab_12", "categorical_8", "rainbow"});
    gui_palette_combo_->setCurrentText(
        QString::fromStdString(config_.gui.colour_palette));
    form->addRow("Colour Palette:", gui_palette_combo_);

    layout->addLayout(form);

    // Render layers
    auto* layersGroup = new QGroupBox("Default Render Layers");
    auto* layersLayout = new QVBoxLayout(layersGroup);

    auto hasLayer = [&](const std::string& name) {
        const auto& layers = config_.gui.default_render_layers;
        return std::find(layers.begin(), layers.end(), name) != layers.end();
    };

    gui_layer_grid_ = new QCheckBox("Grid");
    gui_layer_grid_->setChecked(hasLayer("grid"));
    layersLayout->addWidget(gui_layer_grid_);

    gui_layer_markers_ = new QCheckBox("Markers");
    gui_layer_markers_->setChecked(hasLayer("markers"));
    layersLayout->addWidget(gui_layer_markers_);

    gui_layer_skeleton_ = new QCheckBox("Skeleton");
    gui_layer_skeleton_->setChecked(hasLayer("skeleton"));
    layersLayout->addWidget(gui_layer_skeleton_);

    gui_layer_trails_ = new QCheckBox("Trails");
    gui_layer_trails_->setChecked(hasLayer("trails"));
    layersLayout->addWidget(gui_layer_trails_);

    gui_layer_frustums_ = new QCheckBox("Camera Frustums");
    gui_layer_frustums_->setChecked(hasLayer("camera_frustums"));
    layersLayout->addWidget(gui_layer_frustums_);

    gui_layer_bounding_ = new QCheckBox("Bounding Volumes");
    gui_layer_bounding_->setChecked(hasLayer("bounding_volumes"));
    layersLayout->addWidget(gui_layer_bounding_);

    layout->addWidget(layersGroup);
    layout->addStretch();

    return widget;
}

AppConfig SettingsDialog::result() const {
    AppConfig cfg = config_;

    // Capture
    cfg.capture.target_fps = capture_fps_spin_->value();
    cfg.capture.sync_mode = sync_mode_combo_->currentText().toStdString();
    cfg.capture.max_sync_skew_ms = max_skew_spin_->value();

    // Pose estimation
    cfg.pose_estimation.backend = pe_backend_combo_->currentText().toStdString();
    cfg.pose_estimation.model = pe_model_edit_->text().toStdString();
    cfg.pose_estimation.device = pe_device_combo_->currentText().toStdString();
    cfg.pose_estimation.detection_threshold =
        static_cast<float>(pe_detect_thresh_spin_->value());
    cfg.pose_estimation.keypoint_threshold =
        static_cast<float>(pe_keypoint_thresh_spin_->value());

    // Triangulation
    cfg.triangulation.min_views = tri_min_views_spin_->value();
    cfg.triangulation.ransac_enabled = tri_ransac_check_->isChecked();
    cfg.triangulation.ransac_threshold_px =
        static_cast<float>(tri_ransac_thresh_spin_->value());
    cfg.triangulation.temporal_filter =
        tri_filter_combo_->currentText().toStdString();
    cfg.triangulation.filter_cutoff_hz =
        static_cast<float>(tri_cutoff_spin_->value());

    // Skeleton
    cfg.skeleton.definition = skel_def_combo_->currentText().toStdString();
    cfg.skeleton.ik_solver = skel_solver_combo_->currentText().toStdString();
    cfg.skeleton.joint_limits_enabled = skel_limits_check_->isChecked();

    // GUI
    cfg.gui.canvas_fps = gui_fps_spin_->value();
    cfg.gui.colour_palette = gui_palette_combo_->currentText().toStdString();

    cfg.gui.default_render_layers.clear();
    if (gui_layer_grid_->isChecked()) cfg.gui.default_render_layers.push_back("grid");
    if (gui_layer_markers_->isChecked()) cfg.gui.default_render_layers.push_back("markers");
    if (gui_layer_skeleton_->isChecked()) cfg.gui.default_render_layers.push_back("skeleton");
    if (gui_layer_trails_->isChecked()) cfg.gui.default_render_layers.push_back("trails");
    if (gui_layer_frustums_->isChecked()) cfg.gui.default_render_layers.push_back("camera_frustums");
    if (gui_layer_bounding_->isChecked()) cfg.gui.default_render_layers.push_back("bounding_volumes");

    return cfg;
}

}  // namespace mocap
