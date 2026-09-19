#pragma once

#include "core/types.h"
#include <algorithm>
#include <array>
#include <stdexcept>

namespace mocap {

// COCO-WholeBody body/foot indices -> OpenPose BODY_25. Neck and mid-hip
// are derived from shoulder/hip pairs; use the less confident endpoint.
inline std::vector<Keypoint2D> wholebodyToBody25(const std::vector<Keypoint2D>& source) {
    if (source.size() != 133) throw std::invalid_argument("Expected 133 whole-body joints");
    constexpr std::array<int, 25> indices = {
        0, -1, 6, 8, 10, 5, 7, 9, -1, 12, 14, 16, 11, 13, 15,
        2, 1, 4, 3, 17, 18, 19, 20, 21, 22
    };
    std::vector<Keypoint2D> result(25);
    for (int i = 0; i < 25; ++i) {
        if (indices[i] >= 0) {
            result[i] = source[indices[i]];
        } else {
            const auto& a = source[i == 1 ? 5 : 11];
            const auto& b = source[i == 1 ? 6 : 12];
            result[i].x = (a.x + b.x) * 0.5f;
            result[i].y = (a.y + b.y) * 0.5f;
            result[i].conf = std::min(a.conf, b.conf);
        }
        result[i].index = i;
        result[i].name.clear(); // Caller assigns BODY_25 names.
    }
    return result;
}

} // namespace mocap
