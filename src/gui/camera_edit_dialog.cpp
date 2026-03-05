#include "gui/camera_edit_dialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include <QFileDialog>
#include <QDialogButtonBox>
#include <QGroupBox>

namespace mocap {

CameraEditDialog::CameraEditDialog(const CameraConfig* existing, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(existing ? "Edit Camera" : "Add Camera");
    setMinimumWidth(420);
    setupUi();

    if (existing) {
        populateFromConfig(*existing);
    } else {
        onTypeChanged(type_combo_->currentText());
    }
}

void CameraEditDialog::setupUi() {
    auto* layout = new QVBoxLayout(this);

    auto* form = new QFormLayout;

    id_edit_ = new QLineEdit;
    id_edit_->setPlaceholderText("e.g. cam0");
    form->addRow("Camera ID:", id_edit_);

    type_combo_ = new QComboBox;
    type_combo_->addItems({"usb", "ip", "video"});
    form->addRow("Type:", type_combo_);

    // USB-specific
    device_index_label_ = new QLabel("Device Index:");
    device_index_spin_ = new QSpinBox;
    device_index_spin_->setRange(0, 99);
    form->addRow(device_index_label_, device_index_spin_);

    // IP-specific
    url_label_ = new QLabel("URL:");
    url_edit_ = new QLineEdit;
    url_edit_->setPlaceholderText("rtsp://192.168.1.100:554/stream");
    form->addRow(url_label_, url_edit_);

    // Video file-specific
    file_path_label_ = new QLabel("File Path:");
    auto* fileRow = new QHBoxLayout;
    file_path_edit_ = new QLineEdit;
    file_browse_btn_ = new QPushButton("Browse...");
    fileRow->addWidget(file_path_edit_);
    fileRow->addWidget(file_browse_btn_);
    form->addRow(file_path_label_, fileRow);

    // Resolution
    auto* resGroup = new QGroupBox("Resolution");
    auto* resForm = new QFormLayout(resGroup);
    res_width_spin_ = new QSpinBox;
    res_width_spin_->setRange(160, 7680);
    res_width_spin_->setValue(1920);
    resForm->addRow("Width:", res_width_spin_);
    res_height_spin_ = new QSpinBox;
    res_height_spin_->setRange(120, 4320);
    res_height_spin_->setValue(1080);
    resForm->addRow("Height:", res_height_spin_);

    // Calibration files
    intrinsics_edit_ = new QLineEdit;
    intrinsics_edit_->setPlaceholderText("path/to/intrinsics.yaml (optional)");
    extrinsics_edit_ = new QLineEdit;
    extrinsics_edit_->setPlaceholderText("path/to/extrinsics.json (optional)");

    layout->addLayout(form);
    layout->addWidget(resGroup);

    auto* calibForm = new QFormLayout;
    calibForm->addRow("Intrinsics File:", intrinsics_edit_);
    calibForm->addRow("Extrinsics File:", extrinsics_edit_);
    layout->addLayout(calibForm);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);

    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(type_combo_, &QComboBox::currentTextChanged,
            this, &CameraEditDialog::onTypeChanged);
    connect(file_browse_btn_, &QPushButton::clicked, this, [this]() {
        QString path = QFileDialog::getOpenFileName(
            this, "Select Video File", {},
            "Video Files (*.mp4 *.avi *.mkv *.mov);;All Files (*)");
        if (!path.isEmpty()) {
            file_path_edit_->setText(path);
        }
    });
}

void CameraEditDialog::onTypeChanged(const QString& type) {
    bool isUsb = (type == "usb");
    bool isIp = (type == "ip");
    bool isVideo = (type == "video");

    device_index_label_->setVisible(isUsb);
    device_index_spin_->setVisible(isUsb);
    url_label_->setVisible(isIp);
    url_edit_->setVisible(isIp);
    file_path_label_->setVisible(isVideo);
    file_path_edit_->setVisible(isVideo);
    file_browse_btn_->setVisible(isVideo);
}

void CameraEditDialog::populateFromConfig(const CameraConfig& cfg) {
    id_edit_->setText(QString::fromStdString(cfg.id));
    type_combo_->setCurrentText(QString::fromStdString(cfg.type));
    device_index_spin_->setValue(cfg.device_index);
    url_edit_->setText(QString::fromStdString(cfg.url));
    file_path_edit_->setText(QString::fromStdString(cfg.file_path));
    res_width_spin_->setValue(cfg.resolution_width);
    res_height_spin_->setValue(cfg.resolution_height);
    intrinsics_edit_->setText(QString::fromStdString(cfg.intrinsics_file));
    extrinsics_edit_->setText(QString::fromStdString(cfg.extrinsics_file));
    onTypeChanged(type_combo_->currentText());
}

CameraConfig CameraEditDialog::result() const {
    CameraConfig cfg;
    cfg.id = id_edit_->text().toStdString();
    cfg.type = type_combo_->currentText().toStdString();
    cfg.device_index = device_index_spin_->value();
    cfg.url = url_edit_->text().toStdString();
    cfg.file_path = file_path_edit_->text().toStdString();
    cfg.resolution_width = res_width_spin_->value();
    cfg.resolution_height = res_height_spin_->value();
    cfg.intrinsics_file = intrinsics_edit_->text().toStdString();
    cfg.extrinsics_file = extrinsics_edit_->text().toStdString();
    return cfg;
}

}  // namespace mocap
