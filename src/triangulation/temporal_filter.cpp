#include "triangulation/temporal_filter.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mocap {

// Savitzky-Golay quadratic/cubic smoothing, window=5
const float TemporalFilter::SG_COEFFS[5] = {
    -3.0f / 35.0f, 12.0f / 35.0f, 17.0f / 35.0f, 12.0f / 35.0f, -3.0f / 35.0f
};

TemporalFilter::TemporalFilter(Type type, double cutoff_hz, double sample_rate)
    : type_(type)
    , cutoff_hz_(cutoff_hz)
    , sample_rate_(sample_rate)
{
    computeButterworthCoefficients();
}

void TemporalFilter::computeButterworthCoefficients() {
    // 2nd-order Butterworth low-pass filter using bilinear transform
    double wc = 2.0 * M_PI * cutoff_hz_;
    double T = 1.0 / sample_rate_;

    // Pre-warp
    double wc_warped = (2.0 / T) * std::tan(wc * T / 2.0);

    double K = wc_warped * T / 2.0;
    double K2 = K * K;
    double sqrt2K = std::sqrt(2.0) * K;
    double denom = 1.0 + sqrt2K + K2;

    a0_ = K2 / denom;
    a1_ = 2.0 * K2 / denom;
    a2_ = K2 / denom;
    b1_ = 2.0 * (K2 - 1.0) / denom;
    b2_ = (1.0 - sqrt2K + K2) / denom;
}

float TemporalFilter::filter(float value) {
    double x0 = static_cast<double>(value);
    double y0 = a0_ * x0 + a1_ * x1_ + a2_ * x2_ - b1_ * y1_ - b2_ * y2_;

    x2_ = x1_;
    x1_ = x0;
    y2_ = y1_;
    y1_ = y0;

    return static_cast<float>(y0);
}

std::vector<float> TemporalFilter::filterBatch(const std::vector<float>& values) {
    if (values.empty()) return {};

    if (type_ == Type::SavitzkyGolay) {
        int n = static_cast<int>(values.size());
        std::vector<float> result(n);

        for (int i = 0; i < n; ++i) {
            float sum = 0.0f;
            for (int j = -SG_HALF_WINDOW; j <= SG_HALF_WINDOW; ++j) {
                int idx = std::clamp(i + j, 0, n - 1);
                sum += SG_COEFFS[j + SG_HALF_WINDOW] * values[idx];
            }
            result[i] = sum;
        }
        return result;
    }

    // Butterworth: forward-backward filtering for zero phase shift
    int n = static_cast<int>(values.size());
    std::vector<float> result(n);

    // Forward pass
    reset();
    // Initialize state with first value to reduce transient
    x1_ = x2_ = values[0];
    y1_ = y2_ = values[0];
    for (int i = 0; i < n; ++i) {
        result[i] = filter(values[i]);
    }

    // Backward pass for zero-phase filtering
    std::vector<float> backward(n);
    reset();
    x1_ = x2_ = result[n - 1];
    y1_ = y2_ = result[n - 1];
    for (int i = n - 1; i >= 0; --i) {
        backward[i] = filter(result[i]);
    }

    return backward;
}

std::vector<Vec3f> TemporalFilter::filterBatch3D(const std::vector<Vec3f>& positions) {
    if (positions.empty()) return {};

    int n = static_cast<int>(positions.size());

    // Separate into X, Y, Z channels
    std::vector<float> xs(n), ys(n), zs(n);
    for (int i = 0; i < n; ++i) {
        xs[i] = positions[i].x();
        ys[i] = positions[i].y();
        zs[i] = positions[i].z();
    }

    auto fx = filterBatch(xs);
    auto fy = filterBatch(ys);
    auto fz = filterBatch(zs);

    std::vector<Vec3f> result(n);
    for (int i = 0; i < n; ++i) {
        result[i] = Vec3f(fx[i], fy[i], fz[i]);
    }

    return result;
}

void TemporalFilter::reset() {
    x1_ = x2_ = 0;
    y1_ = y2_ = 0;
}

}  // namespace mocap
