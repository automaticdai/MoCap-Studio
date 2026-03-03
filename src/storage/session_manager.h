#pragma once

#include <string>
#include <vector>

namespace mocap {

class SessionManager {
public:
    struct SessionMeta {
        std::string name;
        std::string created_at;
        double fps = 60.0;
        double duration = 0.0;
        std::vector<std::string> camera_ids;
        std::string skeleton_definition = "body_25";
        int frame_count = 0;
    };

    // Create new session directory with timestamp name
    std::string createSession(const std::string& base_dir, double fps,
                              const std::vector<std::string>& camera_ids);

    // Open existing session
    bool openSession(const std::string& session_dir);

    std::string sessionDir() const;
    SessionMeta metadata() const;

    void setDuration(double duration);
    void setFrameCount(int count);

    // Save/load metadata JSON
    void saveMetadata();
    void loadMetadata();

    // Access data subdirectories
    std::string dataDir() const;
    std::string calibrationDir() const;
    std::string exportDir() const;
    std::string rawVideoDir() const;

    bool isOpen() const;

private:
    std::string session_dir_;
    SessionMeta meta_;
    bool open_ = false;

    void createDirectoryStructure();
    static std::string currentTimestamp();
};

}  // namespace mocap
