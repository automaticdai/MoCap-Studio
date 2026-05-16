#include "pose/onnx_pose_estimator.h"
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <numeric>
#include <algorithm>
#include <chrono>
#include <cmath>

#ifdef MOCAP_USE_DIRECTML
#include <dml_provider_factory.h>
#endif

namespace mocap {

namespace {

struct ProviderAttempt {
    std::string name;  // "cuda", "dml", or "cpu"
    int device_id = 0;
};

// Translate a config "device" string into the ordered list of execution providers
// to try. The list always ends in "cpu" so that initialization succeeds on any host.
std::vector<ProviderAttempt> parseDeviceSpec(const std::string& spec) {
    auto extract_id = [](const std::string& s) {
        auto colon = s.find(':');
        if (colon == std::string::npos) return 0;
        try { return std::stoi(s.substr(colon + 1)); } catch (...) { return 0; }
    };

    std::vector<ProviderAttempt> out;
    if (spec == "auto" || spec.empty()) {
        out.push_back({"cuda", 0});
#ifdef MOCAP_USE_DIRECTML
        out.push_back({"dml", 0});
#endif
    } else if (spec.rfind("cuda", 0) == 0) {
        out.push_back({"cuda", extract_id(spec)});
    } else if (spec.rfind("dml", 0) == 0) {
        out.push_back({"dml", extract_id(spec)});
    } else if (spec != "cpu") {
        spdlog::warn("ONNX Runtime: unknown device spec '{}', using CPU", spec);
    }
    out.push_back({"cpu", 0});
    return out;
}

}  // namespace


const std::vector<std::string> OnnxPoseEstimator::BODY25_NAMES = {
    "nose", "neck", "right_shoulder", "right_elbow", "right_wrist",
    "left_shoulder", "left_elbow", "left_wrist", "mid_hip",
    "right_hip", "right_knee", "right_ankle",
    "left_hip", "left_knee", "left_ankle",
    "right_eye", "left_eye", "right_ear", "left_ear",
    "left_big_toe", "left_small_toe", "left_heel",
    "right_big_toe", "right_small_toe", "right_heel"
};

OnnxPoseEstimator::OnnxPoseEstimator() = default;

OnnxPoseEstimator::~OnnxPoseEstimator() = default;

bool OnnxPoseEstimator::initialize(const std::string& model_path, const std::string& device) {
    try {
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "MoCapPose");
    } catch (const Ort::Exception& e) {
        spdlog::error("ONNX Runtime env init failed: {}", e.what());
        return false;
    }

    // Try each provider in order; first successful session wins. CPU is always last.
    for (const auto& attempt : parseDeviceSpec(device)) {
        try {
            Ort::SessionOptions session_opts;
            session_opts.SetIntraOpNumThreads(4);
            session_opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

            if (attempt.name == "cuda") {
                OrtCUDAProviderOptions cuda_opts;
                cuda_opts.device_id = attempt.device_id;
                session_opts.AppendExecutionProvider_CUDA(cuda_opts);
            } else if (attempt.name == "dml") {
#ifdef MOCAP_USE_DIRECTML
                session_opts.DisableMemPattern();
                session_opts.SetExecutionMode(ORT_SEQUENTIAL);
                Ort::ThrowOnError(
                    OrtSessionOptionsAppendExecutionProvider_DML(session_opts, attempt.device_id));
#else
                continue;  // DirectML not compiled in
#endif
            }
            // For "cpu", append nothing — ORT's default CPU EP is always available.

            session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), session_opts);
            active_provider_ = attempt.name == "cpu"
                ? "cpu"
                : attempt.name + ":" + std::to_string(attempt.device_id);
            spdlog::info("ONNX Runtime: using execution provider '{}'", active_provider_);
            break;
        } catch (const Ort::Exception& e) {
            spdlog::warn("ONNX Runtime: provider '{}' unavailable ({}); trying next",
                         attempt.name, e.what());
            session_.reset();
        }
    }

    if (!session_) {
        spdlog::error("ONNX Runtime: no execution provider could be initialized");
        return false;
    }

    try {
        // Get input info
        size_t num_inputs = session_->GetInputCount();
        input_names_.clear();
        input_name_ptrs_.clear();
        for (size_t i = 0; i < num_inputs; ++i) {
            auto name = session_->GetInputNameAllocated(i, allocator_);
            input_names_.push_back(name.get());
        }
        for (auto& n : input_names_) input_name_ptrs_.push_back(n.c_str());

        // Get input dimensions
        auto input_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if (input_shape.size() == 4) {
            // NCHW format
            input_height_ = static_cast<int>(input_shape[2]);
            input_width_ = static_cast<int>(input_shape[3]);
            if (input_height_ <= 0) input_height_ = 192;
            if (input_width_ <= 0) input_width_ = 256;
        }

        // Get output info
        size_t num_outputs = session_->GetOutputCount();
        output_names_.clear();
        output_name_ptrs_.clear();
        for (size_t i = 0; i < num_outputs; ++i) {
            auto name = session_->GetOutputNameAllocated(i, allocator_);
            output_names_.push_back(name.get());
        }
        for (auto& n : output_names_) output_name_ptrs_.push_back(n.c_str());

        model_name_ = model_path;
        initialized_ = true;

        spdlog::info("ONNX pose model loaded: {} (input: {}x{})",
                     model_path, input_width_, input_height_);
    } catch (const Ort::Exception& e) {
        spdlog::error("ONNX Runtime error: {}", e.what());
        initialized_ = false;
        session_.reset();
        return false;
    }

    warmup();
    return true;
}

void OnnxPoseEstimator::warmup() {
    if (!initialized_) return;
    cv::Mat dummy = cv::Mat::zeros(input_height_, input_width_, CV_8UC3);
    const auto t0 = std::chrono::steady_clock::now();
    estimate(dummy);
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    spdlog::info("ONNX Runtime: warmup inference took {:.1f}ms on '{}'",
                 ms, active_provider_);
    // Don't let the warmup pollute the running average.
    inference_count_ = 0;
    inference_total_ms_ = 0.0;
}

cv::Mat OnnxPoseEstimator::preprocess(const cv::Mat& image) {
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(input_width_, input_height_));

    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    cv::Mat float_img;
    rgb.convertTo(float_img, CV_32F, 1.0 / 255.0);

    // Normalize with ImageNet mean/std
    cv::Mat channels[3];
    cv::split(float_img, channels);
    channels[0] = (channels[0] - 0.485f) / 0.229f;
    channels[1] = (channels[1] - 0.456f) / 0.224f;
    channels[2] = (channels[2] - 0.406f) / 0.225f;

    // Convert HWC to CHW
    int h = input_height_;
    int w = input_width_;
    cv::Mat blob(3 * h, w, CV_32F);
    for (int c = 0; c < 3; ++c) {
        channels[c].copyTo(blob(cv::Rect(0, c * h, w, h)));
    }

    return blob;
}

std::vector<Raw2DPose> OnnxPoseEstimator::estimate(const cv::Mat& image) {
    if (!initialized_ || image.empty()) return {};

    float scale_x = static_cast<float>(image.cols) / input_width_;
    float scale_y = static_cast<float>(image.rows) / input_height_;

    try {
        cv::Mat blob = preprocess(image);

        // Create input tensor
        std::vector<int64_t> input_shape = {1, 3, input_height_, input_width_};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            reinterpret_cast<float*>(blob.data),
            blob.total(),
            input_shape.data(),
            input_shape.size()
        );

        // Run inference (timed for periodic perf logging)
        const auto t0 = std::chrono::steady_clock::now();
        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            input_name_ptrs_.data(),
            &input_tensor,
            1,
            output_name_ptrs_.data(),
            output_name_ptrs_.size()
        );
        const auto t1 = std::chrono::steady_clock::now();

        inference_total_ms_ += std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (++inference_count_ % 60 == 0) {
            spdlog::info("ONNX Runtime: {} avg {:.1f}ms over last 60 inferences",
                         active_provider_, inference_total_ms_ / 60.0);
            inference_total_ms_ = 0.0;
        }

        return postprocess(outputs, scale_x, scale_y);

    } catch (const Ort::Exception& e) {
        spdlog::error("ONNX inference error: {}", e.what());
        return {};
    }
}

std::vector<Raw2DPose> OnnxPoseEstimator::postprocess(
    const std::vector<Ort::Value>& outputs,
    float scale_x, float scale_y
) {
    if (outputs.empty()) return {};

    auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();

    // SimCC-style output: two separate x and y coordinate tensors
    if (outputs.size() >= 2) {
        auto x_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
        auto y_shape = outputs[1].GetTensorTypeAndShapeInfo().GetShape();

        // SimCC outputs have shape [batch, num_keypoints, simcc_len]
        if (x_shape.size() == 3 && y_shape.size() == 3) {
            return decodeSimCC(
                outputs[0].GetTensorData<float>(), x_shape,
                outputs[1].GetTensorData<float>(), y_shape,
                scale_x, scale_y
            );
        }
    }

    // Heatmap-style output: [batch, num_persons * num_keypoints, h, w] or [batch, num_keypoints, h, w]
    if (shape.size() == 4) {
        return decodeHeatmaps(
            outputs[0].GetTensorData<float>(), shape,
            scale_x, scale_y
        );
    }

    spdlog::warn("Unknown output format with {} dimensions", shape.size());
    return {};
}

std::vector<Raw2DPose> OnnxPoseEstimator::decodeHeatmaps(
    const float* heatmap_data,
    const std::vector<int64_t>& heatmap_shape,
    float scale_x, float scale_y
) {
    // Shape: [batch, num_keypoints, height, width]
    int num_kp = static_cast<int>(heatmap_shape[1]);
    int hm_h = static_cast<int>(heatmap_shape[2]);
    int hm_w = static_cast<int>(heatmap_shape[3]);

    int persons = std::max(1, num_kp / num_keypoints_);
    int kp_per_person = (persons > 1) ? num_keypoints_ : num_kp;

    std::vector<Raw2DPose> results;

    for (int p = 0; p < persons; ++p) {
        Raw2DPose pose;
        pose.person_id = p;
        float total_conf = 0.0f;
        int valid_count = 0;

        float min_x = 1e9f, min_y = 1e9f, max_x = -1e9f, max_y = -1e9f;

        for (int k = 0; k < kp_per_person && (p * kp_per_person + k) < num_kp; ++k) {
            int kp_idx = p * kp_per_person + k;
            const float* hm = heatmap_data + kp_idx * hm_h * hm_w;

            // Find max in heatmap
            float max_val = -1.0f;
            int max_pos = 0;
            for (int i = 0; i < hm_h * hm_w; ++i) {
                if (hm[i] > max_val) {
                    max_val = hm[i];
                    max_pos = i;
                }
            }

            float hm_x = static_cast<float>(max_pos % hm_w);
            float hm_y = static_cast<float>(max_pos / hm_w);

            // Scale to image coordinates
            float img_x = (hm_x / hm_w) * input_width_ * scale_x;
            float img_y = (hm_y / hm_h) * input_height_ * scale_y;

            Keypoint2D kp;
            kp.index = k;
            kp.name = (k < static_cast<int>(BODY25_NAMES.size())) ? BODY25_NAMES[k] : "kp_" + std::to_string(k);
            kp.x = img_x;
            kp.y = img_y;
            kp.conf = max_val;

            pose.keypoints.push_back(kp);

            if (max_val >= keypoint_threshold_) {
                total_conf += max_val;
                valid_count++;
                min_x = std::min(min_x, img_x);
                min_y = std::min(min_y, img_y);
                max_x = std::max(max_x, img_x);
                max_y = std::max(max_y, img_y);
            }
        }

        pose.confidence = (valid_count > 0) ? total_conf / valid_count : 0.0f;

        if (valid_count > 0) {
            float pad = 20.0f;
            pose.bbox.x = min_x - pad;
            pose.bbox.y = min_y - pad;
            pose.bbox.width = (max_x - min_x) + 2 * pad;
            pose.bbox.height = (max_y - min_y) + 2 * pad;
        }

        if (pose.confidence >= detection_threshold_) {
            results.push_back(std::move(pose));
        }
    }

    return results;
}

std::vector<Raw2DPose> OnnxPoseEstimator::decodeSimCC(
    const float* x_data, const std::vector<int64_t>& x_shape,
    const float* y_data, const std::vector<int64_t>& y_shape,
    float scale_x, float scale_y
) {
    // Shape: [batch, num_keypoints, simcc_split_len]
    int num_kp = static_cast<int>(x_shape[1]);
    int x_len = static_cast<int>(x_shape[2]);
    int y_len = static_cast<int>(y_shape[2]);

    Raw2DPose pose;
    pose.person_id = 0;
    float total_conf = 0.0f;
    int valid_count = 0;
    float min_x = 1e9f, min_y = 1e9f, max_x = -1e9f, max_y = -1e9f;

    for (int k = 0; k < num_kp; ++k) {
        const float* x_row = x_data + k * x_len;
        const float* y_row = y_data + k * y_len;

        // Find argmax for x and y
        int x_argmax = static_cast<int>(std::max_element(x_row, x_row + x_len) - x_row);
        int y_argmax = static_cast<int>(std::max_element(y_row, y_row + y_len) - y_row);

        float x_conf = x_row[x_argmax];
        float y_conf = y_row[y_argmax];
        float conf = std::min(x_conf, y_conf);

        // SimCC coordinates are typically 2x the input resolution
        float img_x = (static_cast<float>(x_argmax) / x_len) * input_width_ * scale_x;
        float img_y = (static_cast<float>(y_argmax) / y_len) * input_height_ * scale_y;

        Keypoint2D kp;
        kp.index = k;
        kp.name = (k < static_cast<int>(BODY25_NAMES.size())) ? BODY25_NAMES[k] : "kp_" + std::to_string(k);
        kp.x = img_x;
        kp.y = img_y;
        kp.conf = conf;

        pose.keypoints.push_back(kp);

        if (conf >= keypoint_threshold_) {
            total_conf += conf;
            valid_count++;
            min_x = std::min(min_x, img_x);
            min_y = std::min(min_y, img_y);
            max_x = std::max(max_x, img_x);
            max_y = std::max(max_y, img_y);
        }
    }

    pose.confidence = (valid_count > 0) ? total_conf / valid_count : 0.0f;

    if (valid_count > 0) {
        float pad = 20.0f;
        pose.bbox.x = min_x - pad;
        pose.bbox.y = min_y - pad;
        pose.bbox.width = (max_x - min_x) + 2 * pad;
        pose.bbox.height = (max_y - min_y) + 2 * pad;
    }

    std::vector<Raw2DPose> results;
    if (pose.confidence >= detection_threshold_) {
        results.push_back(std::move(pose));
    }
    return results;
}

std::string OnnxPoseEstimator::modelName() const {
    return model_name_;
}

void OnnxPoseEstimator::setDetectionThreshold(float threshold) {
    detection_threshold_ = threshold;
}

void OnnxPoseEstimator::setKeypointThreshold(float threshold) {
    keypoint_threshold_ = threshold;
}

void OnnxPoseEstimator::setNumKeypoints(int num) {
    num_keypoints_ = num;
}

}  // namespace mocap
