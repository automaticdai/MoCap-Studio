#pragma once

#include <QDialog>
#include <QComboBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QLabel>
#include <QFormLayout>
#include "core/config.h"

namespace mocap {

class CameraEditDialog : public QDialog {
    Q_OBJECT
public:
    explicit CameraEditDialog(const CameraConfig* existing = nullptr,
                              QWidget* parent = nullptr);

    CameraConfig result() const;

private slots:
    void onTypeChanged(const QString& type);

private:
    void setupUi();
    void populateFromConfig(const CameraConfig& cfg);

    QLineEdit* id_edit_;
    QComboBox* type_combo_;

    // USB fields
    QLabel* device_index_label_;
    QSpinBox* device_index_spin_;

    // IP fields
    QLabel* url_label_;
    QLineEdit* url_edit_;

    // Video file fields
    QLabel* file_path_label_;
    QLineEdit* file_path_edit_;
    QPushButton* file_browse_btn_;

    // Common fields
    QSpinBox* res_width_spin_;
    QSpinBox* res_height_spin_;
    QLineEdit* intrinsics_edit_;
    QLineEdit* extrinsics_edit_;
};

}  // namespace mocap
