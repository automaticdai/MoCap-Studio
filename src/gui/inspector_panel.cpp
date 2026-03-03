#include "gui/inspector_panel.h"
#include <cmath>

namespace mocap {

InspectorPanel::InspectorPanel(QWidget* parent)
    : QWidget(parent)
{
    skeleton_def_ = SkeletonDefinition::defaultBody25();

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(4);

    selection_label_ = new QLabel("No selection", this);
    selection_label_->setStyleSheet(
        "QLabel { color: #ccc; font-weight: bold; padding: 4px; "
        "background-color: #2a2a30; border-radius: 3px; }");
    layout->addWidget(selection_label_);

    property_tree_ = new QTreeWidget(this);
    property_tree_->setHeaderLabels({"Property", "Value"});
    property_tree_->setColumnWidth(0, 140);
    property_tree_->setAlternatingRowColors(true);
    property_tree_->setStyleSheet(
        "QTreeWidget { background-color: #1e1e24; color: #ccc; border: 1px solid #333; "
        "alternate-background-color: #22222a; }"
        "QTreeWidget::item { padding: 2px; }"
        "QTreeWidget::item:selected { background-color: #3a3a50; }"
        "QHeaderView::section { background-color: #2a2a30; color: #aaa; "
        "border: 1px solid #333; padding: 3px; }");
    layout->addWidget(property_tree_);

    setLayout(layout);
    setMinimumWidth(280);
}

void InspectorPanel::onSkeletonUpdate(const std::vector<SkeletonPose>& skeletons) {
    current_skeletons_ = skeletons;
    updateDisplay();
}

void InspectorPanel::onPose3DUpdate(const std::vector<Pose3D>& poses) {
    current_poses_ = poses;
    updateDisplay();
}

void InspectorPanel::onPersonSelected(int global_person_id) {
    selected_person_ = global_person_id;
    selected_joint_ = -1;
    updateDisplay();
}

void InspectorPanel::onJointSelected(int global_person_id, int joint_index) {
    selected_person_ = global_person_id;
    selected_joint_ = joint_index;
    updateDisplay();
}

void InspectorPanel::updateDisplay() {
    property_tree_->clear();

    if (selected_person_ < 0) {
        selection_label_->setText("No selection");

        // Show scene overview
        auto* scene = addTreeGroup("Scene");
        addTreeItem("Persons", QString::number(current_poses_.size()), scene);

        int total_markers = 0;
        for (const auto& pose : current_poses_) {
            total_markers += static_cast<int>(pose.markers.size());
        }
        addTreeItem("Total Markers", QString::number(total_markers), scene);
        addTreeItem("Skeletons", QString::number(current_skeletons_.size()), scene);

        property_tree_->expandAll();
        return;
    }

    selection_label_->setText(QString("Person %1").arg(selected_person_));

    // Find skeleton for selected person
    const SkeletonPose* skel = nullptr;
    for (const auto& s : current_skeletons_) {
        if (s.global_person_id == selected_person_) {
            skel = &s;
            break;
        }
    }

    // Find pose for selected person
    const Pose3D* pose = nullptr;
    for (const auto& p : current_poses_) {
        if (p.global_person_id == selected_person_) {
            pose = &p;
            break;
        }
    }

    if (selected_joint_ >= 0 && skel) {
        displayJointInfo(*skel, selected_joint_);
    } else if (skel) {
        displayPersonInfo(*skel);
    } else if (pose) {
        displayPoseInfo(*pose);
    }

    property_tree_->expandAll();
}

void InspectorPanel::displayPersonInfo(const SkeletonPose& skeleton) {
    // Root transform
    auto* root = addTreeGroup("Root Transform");
    addTreeItem("X", QString::number(skeleton.root_position.x(), 'f', 3), root);
    addTreeItem("Y", QString::number(skeleton.root_position.y(), 'f', 3), root);
    addTreeItem("Z", QString::number(skeleton.root_position.z(), 'f', 3), root);

    auto* rootRot = addTreeGroup("Root Rotation");
    addTreeItem("W", QString::number(skeleton.root_rotation.w(), 'f', 4), rootRot);
    addTreeItem("X", QString::number(skeleton.root_rotation.x(), 'f', 4), rootRot);
    addTreeItem("Y", QString::number(skeleton.root_rotation.y(), 'f', 4), rootRot);
    addTreeItem("Z", QString::number(skeleton.root_rotation.z(), 'f', 4), rootRot);

    // Joint list
    auto* joints = addTreeGroup("Joints");
    addTreeItem("Count", QString::number(skeleton.joint_rotations.size()), joints);

    for (const auto& jr : skeleton.joint_rotations) {
        QString name = (jr.joint_index < skeleton_def_.jointCount())
            ? QString::fromStdString(skeleton_def_.joint(jr.joint_index).name)
            : QString("Joint %1").arg(jr.joint_index);

        auto* joint = addTreeGroup(name, joints);
        addTreeItem("Euler X", QString::number(jr.euler_degrees.x(), 'f', 1) + "\xC2\xB0", joint);
        addTreeItem("Euler Y", QString::number(jr.euler_degrees.y(), 'f', 1) + "\xC2\xB0", joint);
        addTreeItem("Euler Z", QString::number(jr.euler_degrees.z(), 'f', 1) + "\xC2\xB0", joint);
    }
}

void InspectorPanel::displayPoseInfo(const Pose3D& pose) {
    auto* info = addTreeGroup("3D Pose");
    addTreeItem("Person ID", QString::number(pose.global_person_id), info);
    addTreeItem("Timestamp", QString::number(pose.timestamp, 'f', 3) + " s", info);
    addTreeItem("Markers", QString::number(pose.markers.size()), info);

    // Marker list
    auto* markers = addTreeGroup("Markers");
    for (const auto& m : pose.markers) {
        QString name = (m.index < skeleton_def_.jointCount())
            ? QString::fromStdString(skeleton_def_.joint(m.index).name)
            : QString("Marker %1").arg(m.index);

        auto* marker = addTreeGroup(name, markers);
        addTreeItem("X", QString::number(m.position.x(), 'f', 3) + " m", marker);
        addTreeItem("Y", QString::number(m.position.y(), 'f', 3) + " m", marker);
        addTreeItem("Z", QString::number(m.position.z(), 'f', 3) + " m", marker);
        addTreeItem("Confidence", QString::number(m.confidence, 'f', 2), marker);
    }
}

void InspectorPanel::displayJointInfo(const SkeletonPose& skeleton, int joint_index) {
    QString joint_name = (joint_index < skeleton_def_.jointCount())
        ? QString::fromStdString(skeleton_def_.joint(joint_index).name)
        : QString("Joint %1").arg(joint_index);

    selection_label_->setText(QString("Person %1 / %2").arg(selected_person_).arg(joint_name));

    // Find the joint rotation
    const JointRotation* jr = nullptr;
    for (const auto& r : skeleton.joint_rotations) {
        if (r.joint_index == joint_index) {
            jr = &r;
            break;
        }
    }

    if (!jr) {
        addTreeItem("Status", "No rotation data");
        return;
    }

    // Joint definition
    if (joint_index < skeleton_def_.jointCount()) {
        const auto& jd = skeleton_def_.joint(joint_index);
        auto* def = addTreeGroup("Definition");
        addTreeItem("Name", QString::fromStdString(jd.name), def);
        addTreeItem("Index", QString::number(jd.index), def);
        addTreeItem("Parent", (jd.parent >= 0)
            ? QString::fromStdString(skeleton_def_.joint(jd.parent).name)
            : "None (Root)", def);
    }

    // Rotation
    auto* rot = addTreeGroup("Rotation (Quaternion)");
    addTreeItem("W", QString::number(jr->rotation.w(), 'f', 4), rot);
    addTreeItem("X", QString::number(jr->rotation.x(), 'f', 4), rot);
    addTreeItem("Y", QString::number(jr->rotation.y(), 'f', 4), rot);
    addTreeItem("Z", QString::number(jr->rotation.z(), 'f', 4), rot);

    auto* euler = addTreeGroup("Rotation (Euler)");
    addTreeItem("X", QString::number(jr->euler_degrees.x(), 'f', 1) + "\xC2\xB0", euler);
    addTreeItem("Y", QString::number(jr->euler_degrees.y(), 'f', 1) + "\xC2\xB0", euler);
    addTreeItem("Z", QString::number(jr->euler_degrees.z(), 'f', 1) + "\xC2\xB0", euler);

    // Joint limits
    if (joint_index < skeleton_def_.jointCount()) {
        const auto& limits = skeleton_def_.joint(joint_index).limits;
        auto* lim = addTreeGroup("Joint Limits");
        addTreeItem("Min X", QString::number(limits.min_euler.x(), 'f', 0) + "\xC2\xB0", lim);
        addTreeItem("Max X", QString::number(limits.max_euler.x(), 'f', 0) + "\xC2\xB0", lim);
        addTreeItem("Min Y", QString::number(limits.min_euler.y(), 'f', 0) + "\xC2\xB0", lim);
        addTreeItem("Max Y", QString::number(limits.max_euler.y(), 'f', 0) + "\xC2\xB0", lim);
        addTreeItem("Min Z", QString::number(limits.min_euler.z(), 'f', 0) + "\xC2\xB0", lim);
        addTreeItem("Max Z", QString::number(limits.max_euler.z(), 'f', 0) + "\xC2\xB0", lim);
    }
}

void InspectorPanel::addTreeItem(const QString& name, const QString& value,
                                  QTreeWidgetItem* parent) {
    auto* item = new QTreeWidgetItem();
    item->setText(0, name);
    item->setText(1, value);
    if (parent) {
        parent->addChild(item);
    } else {
        property_tree_->addTopLevelItem(item);
    }
}

QTreeWidgetItem* InspectorPanel::addTreeGroup(const QString& name,
                                                QTreeWidgetItem* parent) {
    auto* item = new QTreeWidgetItem();
    item->setText(0, name);
    item->setFirstColumnSpanned(false);
    QFont font = item->font(0);
    font.setBold(true);
    item->setFont(0, font);
    if (parent) {
        parent->addChild(item);
    } else {
        property_tree_->addTopLevelItem(item);
    }
    return item;
}

}  // namespace mocap
