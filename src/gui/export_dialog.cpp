#include "gui/export_dialog.h"
#include <QDialogButtonBox>
#include <QMessageBox>

namespace mocap {

ExportDialog::ExportDialog(int total_frames, double fps, QWidget* parent)
    : QDialog(parent)
    , total_frames_(total_frames)
    , fps_(fps)
{
    setWindowTitle("Export Data");
    setMinimumWidth(500);

    auto* layout = new QVBoxLayout(this);

    // Format selection
    auto* format_group = new QGroupBox("Export Format", this);
    auto* format_layout = new QHBoxLayout(format_group);

    format_combo_ = new QComboBox(this);
    format_combo_->addItems({"CSV", "JSON", "C3D", "BVH", "FBX", "USD"});
    connect(format_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ExportDialog::onFormatChanged);
    format_layout->addWidget(new QLabel("Format:", this));
    format_layout->addWidget(format_combo_);
    format_layout->addStretch();

    format_group->setLayout(format_layout);
    layout->addWidget(format_group);

    // Data layers
    auto* layer_group = new QGroupBox("Data Layers", this);
    auto* layer_layout = new QVBoxLayout(layer_group);

    l1_check_ = new QCheckBox("L1 - Raw 2D Poses (per-camera keypoints)", this);
    l2_check_ = new QCheckBox("L2 - 3D Marker Positions (triangulated)", this);
    l3_check_ = new QCheckBox("L3 - Skeleton Rotations (joint angles)", this);
    l1_check_->setChecked(false);
    l2_check_->setChecked(true);
    l3_check_->setChecked(true);

    layer_layout->addWidget(l1_check_);
    layer_layout->addWidget(l2_check_);
    layer_layout->addWidget(l3_check_);

    layer_group->setLayout(layer_layout);
    layout->addWidget(layer_group);

    // Frame range
    auto* range_group = new QGroupBox("Frame Range", this);
    auto* range_layout = new QHBoxLayout(range_group);

    range_layout->addWidget(new QLabel("Start:", this));
    start_frame_spin_ = new QSpinBox(this);
    start_frame_spin_->setRange(0, std::max(0, total_frames - 1));
    start_frame_spin_->setValue(0);
    range_layout->addWidget(start_frame_spin_);

    range_layout->addWidget(new QLabel("End:", this));
    end_frame_spin_ = new QSpinBox(this);
    end_frame_spin_->setRange(0, std::max(0, total_frames - 1));
    end_frame_spin_->setValue(std::max(0, total_frames - 1));
    range_layout->addWidget(end_frame_spin_);

    range_layout->addWidget(new QLabel(
        QString("(%1 frames @ %2 fps)").arg(total_frames).arg(fps, 0, 'f', 1), this));
    range_layout->addStretch();

    range_group->setLayout(range_layout);
    layout->addWidget(range_group);

    // Coordinate system
    auto* coord_group = new QGroupBox("Options", this);
    auto* coord_layout = new QHBoxLayout(coord_group);

    coord_layout->addWidget(new QLabel("Coordinate System:", this));
    coord_system_combo_ = new QComboBox(this);
    coord_system_combo_->addItems({
        "Y-Up, Right-Handed (Default)",
        "Z-Up, Right-Handed (Blender/3ds Max)",
        "Y-Up, Left-Handed (Unity)",
    });
    coord_layout->addWidget(coord_system_combo_);
    coord_layout->addStretch();

    coord_group->setLayout(coord_layout);
    layout->addWidget(coord_group);

    // Output directory
    auto* dir_group = new QGroupBox("Output", this);
    auto* dir_layout = new QHBoxLayout(dir_group);

    dir_layout->addWidget(new QLabel("Directory:", this));
    dir_edit_ = new QLineEdit(this);
    dir_edit_->setPlaceholderText("Select export directory...");
    dir_layout->addWidget(dir_edit_);
    auto* browseBtn = new QPushButton("Browse...", this);
    connect(browseBtn, &QPushButton::clicked, this, &ExportDialog::onBrowseDirectory);
    dir_layout->addWidget(browseBtn);

    dir_group->setLayout(dir_layout);
    layout->addWidget(dir_group);

    // Buttons
    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    buttons->button(QDialogButtonBox::Ok)->setText("Export");
    connect(buttons, &QDialogButtonBox::accepted, this, &ExportDialog::onAccept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);

    setLayout(layout);
    setStyleSheet(
        "QDialog { background-color: #2a2a30; }"
        "QLabel { color: #ccc; }"
        "QLineEdit { background-color: #1e1e24; color: #eee; border: 1px solid #444; "
        "border-radius: 3px; padding: 4px; }"
        "QGroupBox { color: #ccc; border: 1px solid #444; border-radius: 4px; "
        "margin-top: 8px; padding-top: 16px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }"
        "QCheckBox { color: #ccc; }"
    );

    updateLayerAvailability();
}

QString ExportDialog::format() const {
    return format_combo_->currentText();
}

bool ExportDialog::exportL1() const {
    return l1_check_->isChecked();
}

bool ExportDialog::exportL2() const {
    return l2_check_->isChecked();
}

bool ExportDialog::exportL3() const {
    return l3_check_->isChecked();
}

int ExportDialog::startFrame() const {
    return start_frame_spin_->value();
}

int ExportDialog::endFrame() const {
    return end_frame_spin_->value();
}

QString ExportDialog::outputDirectory() const {
    return dir_edit_->text();
}

QString ExportDialog::coordinateSystem() const {
    return coord_system_combo_->currentText();
}

void ExportDialog::onBrowseDirectory() {
    QString dir = QFileDialog::getExistingDirectory(this, "Select Export Directory",
                                                     dir_edit_->text());
    if (!dir.isEmpty()) {
        dir_edit_->setText(dir);
    }
}

void ExportDialog::onFormatChanged(int /*index*/) {
    updateLayerAvailability();
}

void ExportDialog::onAccept() {
    if (dir_edit_->text().trimmed().isEmpty()) {
        QMessageBox::warning(this, "Error", "Please select an output directory.");
        return;
    }
    if (!l1_check_->isChecked() && !l2_check_->isChecked() && !l3_check_->isChecked()) {
        QMessageBox::warning(this, "Error", "Please select at least one data layer.");
        return;
    }
    accept();
}

void ExportDialog::updateLayerAvailability() {
    QString fmt = format_combo_->currentText();

    // C3D only supports L2 (3D markers)
    if (fmt == "C3D") {
        l1_check_->setEnabled(false);
        l1_check_->setChecked(false);
        l2_check_->setEnabled(true);
        l2_check_->setChecked(true);
        l3_check_->setEnabled(false);
        l3_check_->setChecked(false);
    }
    // BVH/FBX/USD only support L3 (skeleton)
    else if (fmt == "BVH" || fmt == "FBX" || fmt == "USD") {
        l1_check_->setEnabled(false);
        l1_check_->setChecked(false);
        l2_check_->setEnabled(false);
        l2_check_->setChecked(false);
        l3_check_->setEnabled(true);
        l3_check_->setChecked(true);
    }
    // CSV/JSON support all layers
    else {
        l1_check_->setEnabled(true);
        l2_check_->setEnabled(true);
        l3_check_->setEnabled(true);
    }
}

}  // namespace mocap
