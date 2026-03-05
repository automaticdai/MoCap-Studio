#pragma once

#include <QDialog>
#include <QTabWidget>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include "core/config.h"

namespace mocap {

class SettingsDialog : public QDialog {
    Q_OBJECT
public:
    explicit SettingsDialog(const AppConfig& config, QWidget* parent = nullptr);

    AppConfig result() const;

private:
    void setupUi();
    QWidget* createCaptureTab();
    QWidget* createPoseEstimationTab();
    QWidget* createTriangulationTab();
    QWidget* createSkeletonTab();
    QWidget* createGuiTab();

    AppConfig config_;

    // Capture tab
    QSpinBox* capture_fps_spin_;
    QComboBox* sync_mode_combo_;
    QDoubleSpinBox* max_skew_spin_;

    // Pose estimation tab
    QComboBox* pe_backend_combo_;
    QLineEdit* pe_model_edit_;
    QComboBox* pe_device_combo_;
    QDoubleSpinBox* pe_detect_thresh_spin_;
    QDoubleSpinBox* pe_keypoint_thresh_spin_;

    // Triangulation tab
    QSpinBox* tri_min_views_spin_;
    QCheckBox* tri_ransac_check_;
    QDoubleSpinBox* tri_ransac_thresh_spin_;
    QComboBox* tri_filter_combo_;
    QDoubleSpinBox* tri_cutoff_spin_;

    // Skeleton tab
    QComboBox* skel_def_combo_;
    QComboBox* skel_solver_combo_;
    QCheckBox* skel_limits_check_;

    // GUI tab
    QSpinBox* gui_fps_spin_;
    QComboBox* gui_palette_combo_;
    QCheckBox* gui_layer_grid_;
    QCheckBox* gui_layer_markers_;
    QCheckBox* gui_layer_skeleton_;
    QCheckBox* gui_layer_trails_;
    QCheckBox* gui_layer_frustums_;
    QCheckBox* gui_layer_bounding_;
};

}  // namespace mocap
