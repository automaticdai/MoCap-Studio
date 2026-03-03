#pragma once

#include <string>
#include <vector>
#include "core/types.h"

namespace mocap {

class C3dExporter {
public:
    static void exportPose3D(
        const std::string& path,
        const std::vector<std::pair<double, std::vector<Pose3D>>>& frames,
        double fps,
        int person_id = -1  // -1 = export all persons, >= 0 = specific person
    );
};

}  // namespace mocap
