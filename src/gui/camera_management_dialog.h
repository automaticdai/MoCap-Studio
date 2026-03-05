#pragma once

#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <vector>
#include "core/config.h"

namespace mocap {

class CameraManagementDialog : public QDialog {
    Q_OBJECT
public:
    explicit CameraManagementDialog(const std::vector<CameraConfig>& cameras,
                                    QWidget* parent = nullptr);

    std::vector<CameraConfig> result() const;

private slots:
    void onAdd();
    void onEdit();
    void onRemove();
    void onMoveUp();
    void onMoveDown();
    void onSelectionChanged();

private:
    void setupUi();
    void refreshList();
    QString cameraDisplayText(const CameraConfig& cfg) const;

    QListWidget* list_widget_;
    QPushButton* add_btn_;
    QPushButton* edit_btn_;
    QPushButton* remove_btn_;
    QPushButton* move_up_btn_;
    QPushButton* move_down_btn_;

    std::vector<CameraConfig> cameras_;
};

}  // namespace mocap
