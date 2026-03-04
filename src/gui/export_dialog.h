#pragma once

#include <QDialog>
#include <QComboBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QPushButton>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFileDialog>

namespace mocap {

class ExportDialog : public QDialog {
    Q_OBJECT
public:
    explicit ExportDialog(int total_frames, double fps, QWidget* parent = nullptr);

    // Results
    QString format() const;
    bool exportL1() const;
    bool exportL2() const;
    bool exportL3() const;
    int startFrame() const;
    int endFrame() const;
    QString outputDirectory() const;
    QString coordinateSystem() const;

private slots:
    void onBrowseDirectory();
    void onFormatChanged(int index);
    void onAccept();

private:
    QComboBox* format_combo_;
    QCheckBox* l1_check_;
    QCheckBox* l2_check_;
    QCheckBox* l3_check_;
    QSpinBox* start_frame_spin_;
    QSpinBox* end_frame_spin_;
    QLineEdit* dir_edit_;
    QComboBox* coord_system_combo_;

    int total_frames_;
    double fps_;

    void updateLayerAvailability();
};

}  // namespace mocap
