#include "storage/binary_io.h"
#include <stdexcept>

namespace mocap {

template<typename T>
void BinaryIO::writeVal(std::ofstream& f, const T& val) {
    f.write(reinterpret_cast<const char*>(&val), sizeof(T));
}

template<typename T>
T BinaryIO::readVal(std::ifstream& f) {
    T val;
    f.read(reinterpret_cast<char*>(&val), sizeof(T));
    return val;
}

void BinaryIO::writeString(std::ofstream& f, const std::string& s) {
    uint16_t len = static_cast<uint16_t>(s.size());
    writeVal(f, len);
    f.write(s.data(), len);
}

std::string BinaryIO::readString(std::ifstream& f) {
    uint16_t len = readVal<uint16_t>(f);
    std::string s(len, '\0');
    f.read(s.data(), len);
    return s;
}

void BinaryIO::writeVec3f(std::ofstream& f, const Vec3f& v) {
    writeVal(f, v.x());
    writeVal(f, v.y());
    writeVal(f, v.z());
}

Vec3f BinaryIO::readVec3f(std::ifstream& f) {
    float x = readVal<float>(f);
    float y = readVal<float>(f);
    float z = readVal<float>(f);
    return Vec3f(x, y, z);
}

void BinaryIO::writeQuat(std::ofstream& f, const Quaternion& q) {
    writeVal(f, q.w());
    writeVal(f, q.x());
    writeVal(f, q.y());
    writeVal(f, q.z());
}

Quaternion BinaryIO::readQuat(std::ifstream& f) {
    float w = readVal<float>(f);
    float x = readVal<float>(f);
    float y = readVal<float>(f);
    float z = readVal<float>(f);
    return Quaternion(w, x, y, z);
}

// --- Raw2D ---

void BinaryIO::writeRaw2D(const std::string& path,
    const std::vector<std::pair<double, std::vector<Raw2DPose>>>& frames)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open file for writing: " + path);

    writeVal(f, FramePacket::MAGIC);
    writeVal<uint8_t>(f, 1);  // version
    writeVal<uint8_t>(f, 1);  // layer = Raw2D
    writeVal<uint32_t>(f, static_cast<uint32_t>(frames.size()));

    for (const auto& [timestamp, poses] : frames) {
        writeVal(f, timestamp);
        writeVal<uint16_t>(f, static_cast<uint16_t>(poses.size()));

        for (const auto& pose : poses) {
            writeVal<int32_t>(f, pose.person_id);
            writeVal(f, pose.confidence);
            writeVal(f, pose.bbox.x);
            writeVal(f, pose.bbox.y);
            writeVal(f, pose.bbox.width);
            writeVal(f, pose.bbox.height);

            writeVal<uint16_t>(f, static_cast<uint16_t>(pose.keypoints.size()));
            for (const auto& kp : pose.keypoints) {
                writeString(f, kp.name);
                writeVal<int32_t>(f, kp.index);
                writeVal(f, kp.x);
                writeVal(f, kp.y);
                writeVal(f, kp.conf);
            }
        }
    }
}

std::vector<std::pair<double, std::vector<Raw2DPose>>>
BinaryIO::readRaw2D(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open file for reading: " + path);

    uint32_t magic = readVal<uint32_t>(f);
    if (magic != FramePacket::MAGIC) throw std::runtime_error("Invalid magic number");

    readVal<uint8_t>(f);  // version
    uint8_t layer = readVal<uint8_t>(f);
    if (layer != 1) throw std::runtime_error("Expected layer 1 (Raw2D)");

    uint32_t num_frames = readVal<uint32_t>(f);
    std::vector<std::pair<double, std::vector<Raw2DPose>>> frames;
    frames.reserve(num_frames);

    for (uint32_t fi = 0; fi < num_frames; ++fi) {
        double timestamp = readVal<double>(f);
        uint16_t num_poses = readVal<uint16_t>(f);

        std::vector<Raw2DPose> poses;
        poses.reserve(num_poses);

        for (uint16_t pi = 0; pi < num_poses; ++pi) {
            Raw2DPose pose;
            pose.person_id = readVal<int32_t>(f);
            pose.confidence = readVal<float>(f);
            pose.bbox.x = readVal<float>(f);
            pose.bbox.y = readVal<float>(f);
            pose.bbox.width = readVal<float>(f);
            pose.bbox.height = readVal<float>(f);

            uint16_t num_kp = readVal<uint16_t>(f);
            pose.keypoints.reserve(num_kp);
            for (uint16_t ki = 0; ki < num_kp; ++ki) {
                Keypoint2D kp;
                kp.name = readString(f);
                kp.index = readVal<int32_t>(f);
                kp.x = readVal<float>(f);
                kp.y = readVal<float>(f);
                kp.conf = readVal<float>(f);
                pose.keypoints.push_back(kp);
            }
            poses.push_back(std::move(pose));
        }
        frames.emplace_back(timestamp, std::move(poses));
    }
    return frames;
}

// --- Pose3D ---

void BinaryIO::writePose3D(const std::string& path,
    const std::vector<std::pair<double, std::vector<Pose3D>>>& frames)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open file for writing: " + path);

    writeVal(f, FramePacket::MAGIC);
    writeVal<uint8_t>(f, 1);
    writeVal<uint8_t>(f, 2);  // layer = Pose3D
    writeVal<uint32_t>(f, static_cast<uint32_t>(frames.size()));

    for (const auto& [timestamp, poses] : frames) {
        writeVal(f, timestamp);
        writeVal<uint16_t>(f, static_cast<uint16_t>(poses.size()));

        for (const auto& pose : poses) {
            writeVal<int32_t>(f, pose.global_person_id);
            writeVal<uint16_t>(f, static_cast<uint16_t>(pose.markers.size()));

            for (const auto& m : pose.markers) {
                writeString(f, m.name);
                writeVal<int32_t>(f, m.index);
                writeVec3f(f, m.position);
                writeVal(f, m.confidence);
                writeVal<int32_t>(f, m.n_views);
            }
        }
    }
}

std::vector<std::pair<double, std::vector<Pose3D>>>
BinaryIO::readPose3D(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open file for reading: " + path);

    uint32_t magic = readVal<uint32_t>(f);
    if (magic != FramePacket::MAGIC) throw std::runtime_error("Invalid magic number");

    readVal<uint8_t>(f);
    uint8_t layer = readVal<uint8_t>(f);
    if (layer != 2) throw std::runtime_error("Expected layer 2 (Pose3D)");

    uint32_t num_frames = readVal<uint32_t>(f);
    std::vector<std::pair<double, std::vector<Pose3D>>> frames;
    frames.reserve(num_frames);

    for (uint32_t fi = 0; fi < num_frames; ++fi) {
        double timestamp = readVal<double>(f);
        uint16_t num_poses = readVal<uint16_t>(f);

        std::vector<Pose3D> poses;
        poses.reserve(num_poses);

        for (uint16_t pi = 0; pi < num_poses; ++pi) {
            Pose3D pose;
            pose.global_person_id = readVal<int32_t>(f);
            pose.timestamp = timestamp;

            uint16_t num_markers = readVal<uint16_t>(f);
            pose.markers.reserve(num_markers);
            for (uint16_t mi = 0; mi < num_markers; ++mi) {
                Marker3D m;
                m.name = readString(f);
                m.index = readVal<int32_t>(f);
                m.position = readVec3f(f);
                m.confidence = readVal<float>(f);
                m.n_views = readVal<int32_t>(f);
                pose.markers.push_back(m);
            }
            poses.push_back(std::move(pose));
        }
        frames.emplace_back(timestamp, std::move(poses));
    }
    return frames;
}

// --- Skeleton ---

void BinaryIO::writeSkeleton(const std::string& path,
    const std::vector<std::pair<double, std::vector<SkeletonPose>>>& frames)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open file for writing: " + path);

    writeVal(f, FramePacket::MAGIC);
    writeVal<uint8_t>(f, 1);
    writeVal<uint8_t>(f, 3);  // layer = Skeleton
    writeVal<uint32_t>(f, static_cast<uint32_t>(frames.size()));

    for (const auto& [timestamp, poses] : frames) {
        writeVal(f, timestamp);
        writeVal<uint16_t>(f, static_cast<uint16_t>(poses.size()));

        for (const auto& pose : poses) {
            writeVal<int32_t>(f, pose.global_person_id);
            writeVec3f(f, pose.root_position);
            writeQuat(f, pose.root_rotation);

            writeVal<uint16_t>(f, static_cast<uint16_t>(pose.joint_rotations.size()));
            for (const auto& jr : pose.joint_rotations) {
                writeString(f, jr.joint_name);
                writeVal<int32_t>(f, jr.joint_index);
                writeQuat(f, jr.rotation);
                writeVec3f(f, jr.euler_xyz);
            }
        }
    }
}

std::vector<std::pair<double, std::vector<SkeletonPose>>>
BinaryIO::readSkeleton(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open file for reading: " + path);

    uint32_t magic = readVal<uint32_t>(f);
    if (magic != FramePacket::MAGIC) throw std::runtime_error("Invalid magic number");

    readVal<uint8_t>(f);
    uint8_t layer = readVal<uint8_t>(f);
    if (layer != 3) throw std::runtime_error("Expected layer 3 (Skeleton)");

    uint32_t num_frames = readVal<uint32_t>(f);
    std::vector<std::pair<double, std::vector<SkeletonPose>>> frames;
    frames.reserve(num_frames);

    for (uint32_t fi = 0; fi < num_frames; ++fi) {
        double timestamp = readVal<double>(f);
        uint16_t num_poses = readVal<uint16_t>(f);

        std::vector<SkeletonPose> poses;
        poses.reserve(num_poses);

        for (uint16_t pi = 0; pi < num_poses; ++pi) {
            SkeletonPose pose;
            pose.global_person_id = readVal<int32_t>(f);
            pose.timestamp = timestamp;
            pose.root_position = readVec3f(f);
            pose.root_rotation = readQuat(f);

            uint16_t num_joints = readVal<uint16_t>(f);
            pose.joint_rotations.reserve(num_joints);
            for (uint16_t ji = 0; ji < num_joints; ++ji) {
                JointRotation jr;
                jr.joint_name = readString(f);
                jr.joint_index = readVal<int32_t>(f);
                jr.rotation = readQuat(f);
                jr.euler_xyz = readVec3f(f);
                pose.joint_rotations.push_back(jr);
            }
            poses.push_back(std::move(pose));
        }
        frames.emplace_back(timestamp, std::move(poses));
    }
    return frames;
}

// --- StreamWriter ---

BinaryIO::StreamWriter::StreamWriter(const std::string& path, uint8_t layer) {
    file_.open(path, std::ios::binary);
    if (!file_) throw std::runtime_error("Cannot open file for stream writing: " + path);

    // Write header
    uint32_t magic = FramePacket::MAGIC;
    file_.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    file_.write(reinterpret_cast<const char*>(&layer), sizeof(layer));
}

BinaryIO::StreamWriter::~StreamWriter() {
    close();
}

void BinaryIO::StreamWriter::writePacket(const FramePacket& packet) {
    file_.write(reinterpret_cast<const char*>(&packet.magic), sizeof(packet.magic));
    file_.write(reinterpret_cast<const char*>(&packet.version), sizeof(packet.version));
    file_.write(reinterpret_cast<const char*>(&packet.layer), sizeof(packet.layer));
    file_.write(reinterpret_cast<const char*>(&packet.person_id), sizeof(packet.person_id));
    file_.write(reinterpret_cast<const char*>(&packet.timestamp), sizeof(packet.timestamp));
    file_.write(reinterpret_cast<const char*>(&packet.n_items), sizeof(packet.n_items));

    uint32_t payload_size = static_cast<uint32_t>(packet.payload.size());
    file_.write(reinterpret_cast<const char*>(&payload_size), sizeof(payload_size));
    if (!packet.payload.empty()) {
        file_.write(reinterpret_cast<const char*>(packet.payload.data()), payload_size);
    }
    file_.flush();
}

void BinaryIO::StreamWriter::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

}  // namespace mocap
