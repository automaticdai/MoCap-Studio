#pragma once

#include <string>
#include <vector>
#include <fstream>
#include "core/types.h"

namespace mocap {

class BinaryIO {
public:
    // Write frame data to binary files
    static void writeRaw2D(const std::string& path,
        const std::vector<std::pair<double, std::vector<Raw2DPose>>>& frames);
    static void writePose3D(const std::string& path,
        const std::vector<std::pair<double, std::vector<Pose3D>>>& frames);
    static void writeSkeleton(const std::string& path,
        const std::vector<std::pair<double, std::vector<SkeletonPose>>>& frames);

    // Read back
    static std::vector<std::pair<double, std::vector<Raw2DPose>>>
        readRaw2D(const std::string& path);
    static std::vector<std::pair<double, std::vector<Pose3D>>>
        readPose3D(const std::string& path);
    static std::vector<std::pair<double, std::vector<SkeletonPose>>>
        readSkeleton(const std::string& path);

    // Streaming writer for live recording
    class StreamWriter {
    public:
        StreamWriter(const std::string& path, uint8_t layer);
        ~StreamWriter();
        void writePacket(const FramePacket& packet);
        void close();
    private:
        std::ofstream file_;
    };

private:
    // Helpers for writing/reading primitives
    template<typename T>
    static void writeVal(std::ofstream& f, const T& val);

    template<typename T>
    static T readVal(std::ifstream& f);

    static void writeString(std::ofstream& f, const std::string& s);
    static std::string readString(std::ifstream& f);

    static void writeVec3f(std::ofstream& f, const Vec3f& v);
    static Vec3f readVec3f(std::ifstream& f);

    static void writeQuat(std::ofstream& f, const Quaternion& q);
    static Quaternion readQuat(std::ifstream& f);
};

}  // namespace mocap
