#pragma once
#include "core/stereo_calibrator.h"
#include <opencv2/calib3d.hpp>

struct SyntheticRig {
    mocap::CheckerboardSpec board;
    mocap::CameraIntrinsics first, second;
    mocap::CameraExtrinsics relative;
    std::vector<std::vector<cv::Point2f>> left, right;

    SyntheticRig() {
        first.fx = 1200; first.fy = 1180; first.cx = 960; first.cy = 540;
        first.image_size = {1920, 1080};
        first.distortion_coeffs = (cv::Mat_<double>(1, 5) << -0.12, 0.02, 0.001, -0.001, 0);
        second = first;
        second.fx = 950; second.fy = 970; second.cx = 640; second.cy = 360;
        second.image_size = {1280, 720};
        second.distortion_coeffs = (cv::Mat_<double>(1, 5) << -0.08, 0.01, -0.002, 0.001, 0);
        cv::Rodrigues(cv::Vec3d(0.02, 0.15, -0.03), relative.rotation);
        relative.translation = {-0.35, 0.02, 0.03};
        const auto points = mocap::makeBoardObjectPoints(board);
        for (int i = 0; i < 14; ++i) {
            cv::Vec3d rvec(-0.3 + 0.07 * (i % 7), -0.25 + 0.12 * (i % 5), 0.05 * (i % 3));
            cv::Vec3d tvec(-0.18 + 0.06 * (i % 6), -0.12 + 0.04 * (i % 5), 0.8 + 0.07 * i);
            cv::Mat rotation;
            cv::Rodrigues(rvec, rotation);
            cv::Mat right_rotation = relative.rotation * rotation;
            cv::Mat right_rvec;
            cv::Rodrigues(right_rotation, right_rvec);
            cv::Mat right_tvec = relative.rotation * cv::Mat(tvec) + cv::Mat(relative.translation);
            left.emplace_back(); right.emplace_back();
            cv::projectPoints(points, rvec, tvec, first.cameraMatrix(), first.distortion_coeffs, left.back());
            cv::projectPoints(points, right_rvec, right_tvec, second.cameraMatrix(), second.distortion_coeffs, right.back());
        }
    }
};
