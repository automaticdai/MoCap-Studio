#pragma once

#include <string>
#include <vector>
#include "core/types.h"
#include "core/skeleton_definition.h"

namespace mocap {

class BvhExporter {
public:
    static void exportSkeleton(
        const std::string& path,
        const SkeletonDefinition& skeleton,
        const std::vector<std::pair<double, std::vector<SkeletonPose>>>& frames,
        double fps,
        int person_id = 0
    );

private:
    static void writeJointHierarchy(
        std::ofstream& f,
        const SkeletonDefinition& skeleton,
        int joint_index,
        int depth
    );

    static std::string indent(int depth);
};

}  // namespace mocap
