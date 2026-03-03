#pragma once

#include <vector>
#include "core/types.h"

namespace mocap {

class TemporalFilter {
public:
    enum class Type { Butterworth, SavitzkyGolay };

    TemporalFilter(Type type, double cutoff_hz, double sample_rate);

    // Filter a single value (maintains state for streaming)
    float filter(float value);

    // Filter an entire trajectory (batch mode)
    std::vector<float> filterBatch(const std::vector<float>& values);

    // Filter 3D trajectory
    std::vector<Vec3f> filterBatch3D(const std::vector<Vec3f>& positions);

    void reset();

private:
    Type type_;
    double cutoff_hz_;
    double sample_rate_;

    // Butterworth 2nd-order coefficients
    double a0_, a1_, a2_, b1_, b2_;

    // Filter state
    double x1_ = 0, x2_ = 0;
    double y1_ = 0, y2_ = 0;

    void computeButterworthCoefficients();

    // Savitzky-Golay convolution coefficients (window=5)
    static constexpr int SG_HALF_WINDOW = 2;
    static const float SG_COEFFS[5];
};

}  // namespace mocap
