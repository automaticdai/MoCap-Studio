#pragma once

#include "pose/ipose_estimator.h"
#include <onnxruntime_cxx_api.h>
#include <memory>

namespace mocap {

class OnnxPoseEstimator : public IPoseEstimator {
public:
    OnnxPoseEstimator();
    ~OnnxPoseEstimator() override;

    bool initialize(const std::string& model_path, const std::string& device) override;
    std::vector<Raw2DPose> estimate(const cv::Mat& image) override;
    std::string modelName() const override;

    void setDetectionThreshold(float threshold);
    void setKeypointThreshold(float threshold);
    void setNumKeypoints(int num);

private:
    // Preprocessing
    cv::Mat preprocess(const cv::Mat& image);

    // Postprocessing
    std::vector<Raw2DPose> postprocess(
        const std::vector<Ort::Value>& outputs,
        float scale_x, float scale_y
    );

    // Decode heatmap-style output
    std::vector<Raw2DPose> decodeHeatmaps(
        const float* heatmap_data,
        const std::vector<int64_t>& heatmap_shape,
        float scale_x, float scale_y
    );

    // Decode SimCC (coordinate-based) output
    std::vector<Raw2DPose> decodeSimCC(
        const float* x_data, const std::vector<int64_t>& x_shape,
        const float* y_data, const std::vector<int64_t>& y_shape,
        float scale_x, float scale_y
    );

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;

    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<const char*> input_name_ptrs_;
    std::vector<const char*> output_name_ptrs_;

    int input_width_ = 256;
    int input_height_ = 192;
    int num_keypoints_ = 25;

    float detection_threshold_ = 0.5f;
    float keypoint_threshold_ = 0.3f;

    std::string model_name_;
    bool initialized_ = false;

    static const std::vector<std::string> BODY25_NAMES;
};

}  // namespace mocap
