#pragma once

#include <string>
#include <vector>
#include "core/types.h"
#include "core/skeleton_definition.h"

namespace mocap {

class UsdExporter {
public:
    // Writes a basic USDA text file with skeleton transforms
    static void exportSkeleton(
        const std::string& path,
        const SkeletonDefinition& skeleton,
        const std::vector<std::pair<double, std::vector<SkeletonPose>>>& frames,
        double fps,
        int person_id = 0
    );
};

}  // namespace mocap
