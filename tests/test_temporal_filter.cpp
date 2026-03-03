#include <gtest/gtest.h>
#include "triangulation/temporal_filter.h"
#include <cmath>
#include <numeric>

using namespace mocap;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TEST(TemporalFilterTest, ButterworthReducesNoise) {
    double sample_rate = 60.0;
    double cutoff_hz = 6.0;
    TemporalFilter filt(TemporalFilter::Type::Butterworth, cutoff_hz, sample_rate);

    // Generate a clean 1 Hz sine wave + high-frequency noise (20 Hz)
    int n = 300;
    std::vector<float> noisy(n);
    std::vector<float> clean(n);

    for (int i = 0; i < n; ++i) {
        double t = i / sample_rate;
        clean[i] = static_cast<float>(std::sin(2.0 * M_PI * 1.0 * t));
        noisy[i] = clean[i] + 0.3f * static_cast<float>(std::sin(2.0 * M_PI * 20.0 * t));
    }

    auto filtered = filt.filterBatch(noisy);

    // Compute RMS error between filtered and clean (skip first/last samples for transients)
    float rms_noisy = 0.0f;
    float rms_filtered = 0.0f;
    int start = 30;
    int end = n - 30;

    for (int i = start; i < end; ++i) {
        float err_noisy = noisy[i] - clean[i];
        float err_filtered = filtered[i] - clean[i];
        rms_noisy += err_noisy * err_noisy;
        rms_filtered += err_filtered * err_filtered;
    }

    rms_noisy = std::sqrt(rms_noisy / (end - start));
    rms_filtered = std::sqrt(rms_filtered / (end - start));

    // Filtered signal should be much closer to clean than noisy
    EXPECT_LT(rms_filtered, rms_noisy * 0.5f);
}

TEST(TemporalFilterTest, ButterworthPreservesLowFrequency) {
    double sample_rate = 60.0;
    double cutoff_hz = 10.0;
    TemporalFilter filt(TemporalFilter::Type::Butterworth, cutoff_hz, sample_rate);

    // Pure 1 Hz sine wave (well below cutoff)
    int n = 300;
    std::vector<float> signal(n);
    for (int i = 0; i < n; ++i) {
        signal[i] = static_cast<float>(std::sin(2.0 * M_PI * 1.0 * i / sample_rate));
    }

    auto filtered = filt.filterBatch(signal);

    // Should be very close to original (skip transients)
    for (int i = 60; i < n - 60; ++i) {
        EXPECT_NEAR(filtered[i], signal[i], 0.1f)
            << "Mismatch at sample " << i;
    }
}

TEST(TemporalFilterTest, ButterworthAttenuatesHighFrequency) {
    double sample_rate = 60.0;
    double cutoff_hz = 5.0;
    TemporalFilter filt(TemporalFilter::Type::Butterworth, cutoff_hz, sample_rate);

    // Pure 25 Hz sine wave (well above cutoff)
    int n = 300;
    std::vector<float> signal(n);
    for (int i = 0; i < n; ++i) {
        signal[i] = static_cast<float>(std::sin(2.0 * M_PI * 25.0 * i / sample_rate));
    }

    auto filtered = filt.filterBatch(signal);

    // Amplitude should be significantly reduced
    float max_amp = 0.0f;
    for (int i = 60; i < n - 60; ++i) {
        max_amp = std::max(max_amp, std::abs(filtered[i]));
    }

    EXPECT_LT(max_amp, 0.2f);  // should be heavily attenuated
}

TEST(TemporalFilterTest, Filter3DTrajectory) {
    double sample_rate = 60.0;
    double cutoff_hz = 6.0;
    TemporalFilter filt(TemporalFilter::Type::Butterworth, cutoff_hz, sample_rate);

    int n = 120;
    std::vector<Vec3f> trajectory(n);
    for (int i = 0; i < n; ++i) {
        double t = i / sample_rate;
        // Smooth circular motion + noise
        float x = static_cast<float>(std::cos(2.0 * M_PI * 0.5 * t));
        float y = 1.5f;
        float z = static_cast<float>(std::sin(2.0 * M_PI * 0.5 * t));

        // Add noise
        float noise = 0.1f * static_cast<float>(std::sin(2.0 * M_PI * 20.0 * t));
        trajectory[i] = Vec3f(x + noise, y + noise * 0.5f, z + noise);
    }

    auto filtered = filt.filterBatch3D(trajectory);
    EXPECT_EQ(filtered.size(), trajectory.size());

    // Y values should be close to 1.5 (constant + noise removed)
    for (int i = 30; i < n - 30; ++i) {
        EXPECT_NEAR(filtered[i].y(), 1.5f, 0.15f);
    }
}

TEST(TemporalFilterTest, SavitzkyGolaySmoothing) {
    double sample_rate = 60.0;
    TemporalFilter filt(TemporalFilter::Type::SavitzkyGolay, 6.0, sample_rate);

    // Step function with noise
    int n = 50;
    std::vector<float> signal(n);
    for (int i = 0; i < n; ++i) {
        signal[i] = (i < 25) ? 0.0f : 1.0f;
        signal[i] += 0.05f * ((i % 3) - 1);  // small noise
    }

    auto filtered = filt.filterBatch(signal);
    EXPECT_EQ(filtered.size(), signal.size());

    // Far from edge, should be smooth
    EXPECT_NEAR(filtered[5], 0.0f, 0.15f);
    EXPECT_NEAR(filtered[40], 1.0f, 0.15f);
}

TEST(TemporalFilterTest, EmptyInput) {
    TemporalFilter filt(TemporalFilter::Type::Butterworth, 6.0, 60.0);

    auto result = filt.filterBatch({});
    EXPECT_TRUE(result.empty());

    auto result3d = filt.filterBatch3D({});
    EXPECT_TRUE(result3d.empty());
}

TEST(TemporalFilterTest, Reset) {
    TemporalFilter filt(TemporalFilter::Type::Butterworth, 6.0, 60.0);

    // Filter some values
    filt.filter(1.0f);
    filt.filter(2.0f);

    // Reset and filter same value — should not depend on history
    filt.reset();
    float v1 = filt.filter(5.0f);

    filt.reset();
    float v2 = filt.filter(5.0f);

    EXPECT_FLOAT_EQ(v1, v2);
}
