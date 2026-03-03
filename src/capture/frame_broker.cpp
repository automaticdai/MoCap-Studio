#include "capture/frame_broker.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>

namespace mocap {

FrameBroker::FrameBroker(QObject* parent)
    : QObject(parent)
{}

FrameBroker::~FrameBroker() {
    stop();
}

void FrameBroker::addCamera(std::shared_ptr<ICameraSource> source) {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    auto slot = std::make_unique<CameraSlot>();
    slot->source = std::move(source);
    camera_slots_.push_back(std::move(slot));
}

void FrameBroker::removeCamera(const std::string& camera_id) {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    auto it = std::remove_if(camera_slots_.begin(), camera_slots_.end(),
        [&](const std::unique_ptr<CameraSlot>& slot) {
            return slot->source->id() == camera_id;
        });

    for (auto i = it; i != camera_slots_.end(); ++i) {
        (*i)->running = false;
        if ((*i)->thread.joinable()) {
            (*i)->buffer_cv.notify_all();
            (*i)->thread.join();
        }
    }
    camera_slots_.erase(it, camera_slots_.end());
}

void FrameBroker::start() {
    if (running_.load()) return;
    running_ = true;

    std::lock_guard<std::mutex> lock(slots_mutex_);
    for (auto& slot : camera_slots_) {
        slot->running = true;
        slot->thread = std::thread(&FrameBroker::cameraThreadFunc, this, slot.get());
    }

    sync_thread_ = std::thread(&FrameBroker::syncThreadFunc, this);
    spdlog::info("FrameBroker started with {} cameras", camera_slots_.size());
}

void FrameBroker::stop() {
    if (!running_.load()) return;
    running_ = false;

    std::lock_guard<std::mutex> lock(slots_mutex_);
    for (auto& slot : camera_slots_) {
        slot->running = false;
        slot->buffer_cv.notify_all();
        if (slot->thread.joinable()) slot->thread.join();
    }

    if (sync_thread_.joinable()) sync_thread_.join();
    spdlog::info("FrameBroker stopped");
}

bool FrameBroker::isRunning() const {
    return running_.load();
}

int FrameBroker::cameraCount() const {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    return static_cast<int>(camera_slots_.size());
}

void FrameBroker::setMaxSyncSkewMs(double ms) {
    max_sync_skew_ms_ = ms;
}

double FrameBroker::maxSyncSkewMs() const {
    return max_sync_skew_ms_;
}

void FrameBroker::cameraThreadFunc(CameraSlot* slot) {
    while (slot->running.load()) {
        CapturedFrame frame;
        if (slot->source->grabFrame(frame, 100)) {
            std::lock_guard<std::mutex> lock(slot->buffer_mutex);
            slot->buffer.push_back(std::move(frame));
            while (static_cast<int>(slot->buffer.size()) > BUFFER_SIZE) {
                slot->buffer.pop_front();
            }
            slot->buffer_cv.notify_one();
        } else if (!slot->source->isOpened()) {
            emit cameraError(
                QString::fromStdString(slot->source->id()),
                "Camera disconnected"
            );
            break;
        }
    }
}

void FrameBroker::syncThreadFunc() {
    while (running_.load()) {
        // Collect one frame from each camera
        std::vector<CapturedFrame> collected;
        bool all_available = true;

        {
            std::lock_guard<std::mutex> lock(slots_mutex_);
            if (camera_slots_.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            for (auto& slot : camera_slots_) {
                std::unique_lock<std::mutex> buf_lock(slot->buffer_mutex);
                if (slot->buffer.empty()) {
                    // Wait briefly for a frame to arrive
                    slot->buffer_cv.wait_for(buf_lock, std::chrono::milliseconds(16));
                }
                if (!slot->buffer.empty()) {
                    collected.push_back(std::move(slot->buffer.front()));
                    slot->buffer.pop_front();
                } else {
                    all_available = false;
                }
            }
        }

        if (!all_available || collected.empty()) {
            continue;
        }

        // Check timestamp sync: find min and max timestamps
        double min_ts = collected[0].timestamp;
        double max_ts = collected[0].timestamp;
        for (const auto& f : collected) {
            min_ts = std::min(min_ts, f.timestamp);
            max_ts = std::max(max_ts, f.timestamp);
        }

        double skew_ms = (max_ts - min_ts) * 1000.0;

        // If within tolerance, emit as a synced frame set
        if (skew_ms <= max_sync_skew_ms_ || collected.size() == 1) {
            auto fs = std::make_shared<FrameSet>();
            fs->timestamp = (min_ts + max_ts) / 2.0;
            fs->frames = std::move(collected);
            emit frameSetReady(fs);
        } else {
            spdlog::debug("Frame sync skew too large: {:.1f} ms (max: {:.1f} ms)",
                         skew_ms, max_sync_skew_ms_);
            // Discard oldest frames and retry on next iteration
        }
    }
}

}  // namespace mocap
