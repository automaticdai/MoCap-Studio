#include "gui/camera_management_dialog.h"
#include "gui/camera_edit_dialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialogButtonBox>
#include <QMessageBox>

namespace mocap {

CameraManagementDialog::CameraManagementDialog(
    const std::vector<CameraConfig>& cameras, QWidget* parent)
    : QDialog(parent), cameras_(cameras)
{
    setWindowTitle("Camera Setup");
    setMinimumSize(500, 400);
    setupUi();
    refreshList();
    onSelectionChanged();
}

void CameraManagementDialog::setupUi() {
    auto* layout = new QHBoxLayout(this);

    // List on the left
    list_widget_ = new QListWidget;
    layout->addWidget(list_widget_, 1);

    // Buttons on the right
    auto* btnLayout = new QVBoxLayout;

    add_btn_ = new QPushButton("Add...");
    edit_btn_ = new QPushButton("Edit...");
    remove_btn_ = new QPushButton("Remove");
    move_up_btn_ = new QPushButton("Move Up");
    move_down_btn_ = new QPushButton("Move Down");

    btnLayout->addWidget(add_btn_);
    btnLayout->addWidget(edit_btn_);
    btnLayout->addWidget(remove_btn_);
    btnLayout->addSpacing(16);
    btnLayout->addWidget(move_up_btn_);
    btnLayout->addWidget(move_down_btn_);
    btnLayout->addStretch();

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    btnLayout->addWidget(buttons);

    layout->addLayout(btnLayout);

    connect(add_btn_, &QPushButton::clicked, this, &CameraManagementDialog::onAdd);
    connect(edit_btn_, &QPushButton::clicked, this, &CameraManagementDialog::onEdit);
    connect(remove_btn_, &QPushButton::clicked, this, &CameraManagementDialog::onRemove);
    connect(move_up_btn_, &QPushButton::clicked, this, &CameraManagementDialog::onMoveUp);
    connect(move_down_btn_, &QPushButton::clicked, this, &CameraManagementDialog::onMoveDown);
    connect(list_widget_, &QListWidget::currentRowChanged,
            this, &CameraManagementDialog::onSelectionChanged);
    connect(list_widget_, &QListWidget::itemDoubleClicked,
            this, &CameraManagementDialog::onEdit);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

void CameraManagementDialog::refreshList() {
    int row = list_widget_->currentRow();
    list_widget_->clear();
    for (const auto& cam : cameras_) {
        list_widget_->addItem(cameraDisplayText(cam));
    }
    if (row >= 0 && row < list_widget_->count()) {
        list_widget_->setCurrentRow(row);
    }
}

QString CameraManagementDialog::cameraDisplayText(const CameraConfig& cfg) const {
    QString text = QString::fromStdString(cfg.id) + " [" +
                   QString::fromStdString(cfg.type) + "]";
    if (cfg.type == "usb") {
        text += QString(" — device %1").arg(cfg.device_index);
    } else if (cfg.type == "ip") {
        text += " — " + QString::fromStdString(cfg.url);
    } else if (cfg.type == "video") {
        text += " — " + QString::fromStdString(cfg.file_path);
    }
    return text;
}

void CameraManagementDialog::onAdd() {
    CameraEditDialog dialog(nullptr, this);
    if (dialog.exec() == QDialog::Accepted) {
        cameras_.push_back(dialog.result());
        refreshList();
        list_widget_->setCurrentRow(list_widget_->count() - 1);
    }
}

void CameraManagementDialog::onEdit() {
    int row = list_widget_->currentRow();
    if (row < 0 || row >= static_cast<int>(cameras_.size())) return;

    CameraEditDialog dialog(&cameras_[row], this);
    if (dialog.exec() == QDialog::Accepted) {
        cameras_[row] = dialog.result();
        refreshList();
    }
}

void CameraManagementDialog::onRemove() {
    int row = list_widget_->currentRow();
    if (row < 0 || row >= static_cast<int>(cameras_.size())) return;

    auto reply = QMessageBox::question(
        this, "Remove Camera",
        QString("Remove camera '%1'?").arg(
            QString::fromStdString(cameras_[row].id)),
        QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        cameras_.erase(cameras_.begin() + row);
        refreshList();
    }
}

void CameraManagementDialog::onMoveUp() {
    int row = list_widget_->currentRow();
    if (row <= 0) return;
    std::swap(cameras_[row], cameras_[row - 1]);
    refreshList();
    list_widget_->setCurrentRow(row - 1);
}

void CameraManagementDialog::onMoveDown() {
    int row = list_widget_->currentRow();
    if (row < 0 || row >= static_cast<int>(cameras_.size()) - 1) return;
    std::swap(cameras_[row], cameras_[row + 1]);
    refreshList();
    list_widget_->setCurrentRow(row + 1);
}

void CameraManagementDialog::onSelectionChanged() {
    bool hasSelection = list_widget_->currentRow() >= 0;
    int row = list_widget_->currentRow();
    int count = static_cast<int>(cameras_.size());

    edit_btn_->setEnabled(hasSelection);
    remove_btn_->setEnabled(hasSelection);
    move_up_btn_->setEnabled(hasSelection && row > 0);
    move_down_btn_->setEnabled(hasSelection && row < count - 1);
}

std::vector<CameraConfig> CameraManagementDialog::result() const {
    return cameras_;
}

}  // namespace mocap
