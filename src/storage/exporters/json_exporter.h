#pragma once

#include <string>
#include <vector>
#include "core/types.h"

namespace mocap {

class JsonExporter {
public:
    static void exportRaw2D(const std::string& path,
        const std::vector<std::pair<double, std::vector<Raw2DPose>>>& frames);
    static void exportPose3D(const std::string& path,
        const std::vector<std::pair<double, std::vector<Pose3D>>>& frames);
    static void exportSkeleton(const std::string& path,
        const std::vector<std::pair<double, std::vector<SkeletonPose>>>& frames);
};

}  // namespace mocap
