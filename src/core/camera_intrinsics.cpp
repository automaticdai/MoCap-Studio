#include "core/camera_intrinsics.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

namespace mocap {

cv::Mat CameraIntrinsics::cameraMatrix() const {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    return K;
}

cv::Point2f CameraIntrinsics::undistort(const cv::Point2f& pt) const {
    std::vector<cv::Point2f> src = {pt};
    std::vector<cv::Point2f> dst;
    cv::Mat K = cameraMatrix();
    cv::undistortPoints(src, dst, K, distortion_coeffs, cv::noArray(), K);
    return dst[0];
}

CameraIntrinsics CameraIntrinsics::loadFromYaml(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    CameraIntrinsics intr;

    if (!fs.isOpened()) return intr;

    fs["fx"] >> intr.fx;
    fs["fy"] >> intr.fy;
    fs["cx"] >> intr.cx;
    fs["cy"] >> intr.cy;
    fs["distortion_coefficients"] >> intr.distortion_coeffs;

    int w = 0, h = 0;
    fs["image_width"] >> w;
    fs["image_height"] >> h;
    intr.image_size = cv::Size(w, h);

    return intr;
}

void CameraIntrinsics::saveToYaml(const std::string& path) const {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    fs << "fx" << fx;
    fs << "fy" << fy;
    fs << "cx" << cx;
    fs << "cy" << cy;
    fs << "distortion_coefficients" << distortion_coeffs;
    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;
}

CameraExtrinsics::CameraExtrinsics()
    : rotation(cv::Mat::eye(3, 3, CV_64F))
    , translation(0.0, 0.0, 0.0)
{}

cv::Mat CameraExtrinsics::projectionMatrix(const CameraIntrinsics& intr) const {
    cv::Mat K = intr.cameraMatrix();
    cv::Mat Rt(3, 4, CV_64F);
    rotation.copyTo(Rt(cv::Rect(0, 0, 3, 3)));
    Rt.at<double>(0, 3) = translation[0];
    Rt.at<double>(1, 3) = translation[1];
    Rt.at<double>(2, 3) = translation[2];
    return K * Rt;
}

CameraExtrinsics CameraExtrinsics::loadFromJson(const std::string& path) {
    std::ifstream f(path);
    nlohmann::json j;
    f >> j;

    CameraExtrinsics ext;

    if (j.contains("rotation")) {
        auto& r = j["rotation"];
        ext.rotation = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k)
                ext.rotation.at<double>(i, k) = r[i * 3 + k].get<double>();
    }

    if (j.contains("translation")) {
        auto& t = j["translation"];
        ext.translation = cv::Vec3d(t[0].get<double>(), t[1].get<double>(), t[2].get<double>());
    }

    return ext;
}

void CameraExtrinsics::saveToJson(const std::string& path) const {
    nlohmann::json j;

    std::vector<double> r(9);
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k)
            r[i * 3 + k] = rotation.at<double>(i, k);
    j["rotation"] = r;

    j["translation"] = {translation[0], translation[1], translation[2]};

    std::ofstream f(path);
    f << j.dump(2);
}

}  // namespace mocap
