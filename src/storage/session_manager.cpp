#include "storage/session_manager.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace mocap {

std::string SessionManager::createSession(
    const std::string& base_dir, double fps,
    const std::vector<std::string>& camera_ids
) {
    std::string timestamp = currentTimestamp();
    std::string dir_name = "session_" + timestamp;
    session_dir_ = (fs::path(base_dir) / dir_name).string();

    meta_.name = dir_name;
    meta_.created_at = timestamp;
    meta_.fps = fps;
    meta_.duration = 0.0;
    meta_.camera_ids = camera_ids;
    meta_.frame_count = 0;

    createDirectoryStructure();
    saveMetadata();

    open_ = true;
    spdlog::info("Created session: {}", session_dir_);
    return session_dir_;
}

bool SessionManager::openSession(const std::string& session_dir) {
    if (!fs::exists(session_dir)) {
        spdlog::error("Session directory does not exist: {}", session_dir);
        return false;
    }

    session_dir_ = session_dir;
    try {
        loadMetadata();
        open_ = true;
        spdlog::info("Opened session: {}", session_dir_);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Failed to open session: {}", e.what());
        return false;
    }
}

std::string SessionManager::sessionDir() const { return session_dir_; }
SessionManager::SessionMeta SessionManager::metadata() const { return meta_; }
bool SessionManager::isOpen() const { return open_; }

void SessionManager::setDuration(double duration) { meta_.duration = duration; }
void SessionManager::setFrameCount(int count) { meta_.frame_count = count; }

std::string SessionManager::dataDir() const {
    return (fs::path(session_dir_) / "data").string();
}

std::string SessionManager::calibrationDir() const {
    return (fs::path(session_dir_) / "calibration").string();
}

std::string SessionManager::exportDir() const {
    return (fs::path(session_dir_) / "exports").string();
}

std::string SessionManager::rawVideoDir() const {
    return (fs::path(session_dir_) / "raw_video").string();
}

void SessionManager::saveMetadata() {
    nlohmann::json j;
    j["name"] = meta_.name;
    j["created_at"] = meta_.created_at;
    j["fps"] = meta_.fps;
    j["duration"] = meta_.duration;
    j["camera_ids"] = meta_.camera_ids;
    j["skeleton_definition"] = meta_.skeleton_definition;
    j["frame_count"] = meta_.frame_count;

    std::string path = (fs::path(session_dir_) / "meta.json").string();
    std::ofstream f(path);
    f << j.dump(2);
}

void SessionManager::loadMetadata() {
    std::string path = (fs::path(session_dir_) / "meta.json").string();
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open meta.json in " + session_dir_);
    }

    nlohmann::json j;
    f >> j;

    meta_.name = j.value("name", "");
    meta_.created_at = j.value("created_at", "");
    meta_.fps = j.value("fps", 60.0);
    meta_.duration = j.value("duration", 0.0);
    meta_.skeleton_definition = j.value("skeleton_definition", "body_25");
    meta_.frame_count = j.value("frame_count", 0);

    if (j.contains("camera_ids")) {
        meta_.camera_ids = j["camera_ids"].get<std::vector<std::string>>();
    }
}

void SessionManager::createDirectoryStructure() {
    fs::create_directories(session_dir_);
    fs::create_directories(dataDir());
    fs::create_directories(calibrationDir());
    fs::create_directories((fs::path(calibrationDir()) / "intrinsics").string());
    fs::create_directories(exportDir());
    fs::create_directories(rawVideoDir());
}

std::string SessionManager::currentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&time_t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

}  // namespace mocap
