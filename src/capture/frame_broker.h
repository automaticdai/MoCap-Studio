#pragma once

#include <QObject>
#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#include <condition_variable>
#include "capture/icamera_source.h"
#include "core/types.h"

namespace mocap {

class FrameBroker : public QObject {
    Q_OBJECT
public:
    explicit FrameBroker(QObject* parent = nullptr);
    ~FrameBroker() override;

    void addCamera(std::shared_ptr<ICameraSource> source);
    void removeCamera(const std::string& camera_id);

    void start();
    void stop();

    bool isRunning() const;
    int cameraCount() const;

    void setMaxSyncSkewMs(double ms);
    double maxSyncSkewMs() const;

signals:
    void frameSetReady(std::shared_ptr<FrameSet> frameSet);
    void cameraError(const QString& camera_id, const QString& error);

private:
    static constexpr int BUFFER_SIZE = 8;

    struct CameraSlot {
        std::shared_ptr<ICameraSource> source;
        std::thread thread;
        std::atomic<bool> running{false};

        std::mutex buffer_mutex;
        std::deque<CapturedFrame> buffer;
        std::condition_variable buffer_cv;
    };

    void cameraThreadFunc(CameraSlot* slot);
    void syncThreadFunc();

    std::vector<std::unique_ptr<CameraSlot>> camera_slots_;
    std::thread sync_thread_;
    std::atomic<bool> running_{false};
    double max_sync_skew_ms_ = 5.0;
    mutable std::mutex slots_mutex_;
};

}  // namespace mocap
