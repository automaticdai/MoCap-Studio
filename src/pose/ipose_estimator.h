#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "core/types.h"

namespace mocap {

class IPoseEstimator {
public:
    virtual ~IPoseEstimator() = default;
    virtual bool initialize(const std::string& model_path, const std::string& device) = 0;
    virtual std::vector<Raw2DPose> estimate(const cv::Mat& image) = 0;
    virtual std::string modelName() const = 0;
};

}  // namespace mocap
