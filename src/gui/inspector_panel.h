#pragma once

#include <QWidget>
#include <QTreeWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <vector>
#include "core/types.h"
#include "core/skeleton_definition.h"

namespace mocap {

class InspectorPanel : public QWidget {
    Q_OBJECT
public:
    explicit InspectorPanel(QWidget* parent = nullptr);

public slots:
    void onSkeletonUpdate(const std::vector<SkeletonPose>& skeletons);
    void onPose3DUpdate(const std::vector<Pose3D>& poses);
    void onPersonSelected(int global_person_id);
    void onJointSelected(int global_person_id, int joint_index);

private:
    QTreeWidget* property_tree_;
    QLabel* selection_label_;

    int selected_person_ = -1;
    int selected_joint_ = -1;
    std::vector<SkeletonPose> current_skeletons_;
    std::vector<Pose3D> current_poses_;
    SkeletonDefinition skeleton_def_;

    void updateDisplay();
    void displayPersonInfo(const SkeletonPose& skeleton);
    void displayPoseInfo(const Pose3D& pose);
    void displayJointInfo(const SkeletonPose& skeleton, int joint_index);

    void addTreeItem(const QString& name, const QString& value,
                     QTreeWidgetItem* parent = nullptr);
    QTreeWidgetItem* addTreeGroup(const QString& name,
                                   QTreeWidgetItem* parent = nullptr);
};

}  // namespace mocap
