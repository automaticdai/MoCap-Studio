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
    auto slot = std::make_shared<CameraSlot>();
    slot->source = std::move(source);
    camera_slots_.push_back(std::move(slot));
}

void FrameBroker::removeCamera(const std::string& camera_id) {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    for (auto it = camera_slots_.begin(); it != camera_slots_.end();) {
        if ((*it)->source->id() != camera_id) { ++it; continue; }
        (*it)->running = false;
        (*it)->buffer_cv.notify_all();
        if ((*it)->thread.joinable()) {
            (*it)->thread.join();
        }
        it = camera_slots_.erase(it);
    }
}

void FrameBroker::start(bool preview_only) {
    if (running_.load()) return;
    running_ = true;

    std::lock_guard<std::mutex> lock(slots_mutex_);
    latest_delivery_ = std::any_of(camera_slots_.begin(), camera_slots_.end(),
        [](const auto& slot) { return slot->source->prefersLatestFrame(); });
    for (auto& slot : camera_slots_) {
        slot->buffer.clear();
        if (preview_only && !slot->source->isLive()) continue;
        slot->running = true;
        slot->thread = std::thread(&FrameBroker::cameraThreadFunc, this, slot.get());
    }

    sync_thread_ = std::thread(&FrameBroker::syncThreadFunc, this);
    spdlog::info("FrameBroker started with {} cameras", camera_slots_.size());
}

void FrameBroker::stop() {
    if (!running_.load()) return;
    running_ = false;

    std::unique_lock<std::mutex> lock(slots_mutex_);
    for (auto& slot : camera_slots_) {
        slot->running = false;
        slot->buffer_cv.notify_all();
        if (slot->thread.joinable()) slot->thread.join();
    }

    // The sync thread may be waiting for slots_mutex_.
    lock.unlock();
    if (sync_thread_.joinable()) sync_thread_.join();
    {
        std::lock_guard<std::mutex> delivery_lock(delivery_mutex_);
        pending_frame_set_.reset();
    }
    spdlog::info("FrameBroker stopped");
}

bool FrameBroker::isRunning() const {
    return running_.load();
}

int FrameBroker::cameraCount() const {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    return static_cast<int>(std::count_if(camera_slots_.begin(), camera_slots_.end(),
        [](const auto& slot) { return slot->source->isOpened(); }));
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
            if (slot->source->prefersLatestFrame()) slot->buffer.clear();
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

        std::vector<std::shared_ptr<CameraSlot>> camera_snapshot;
        {
            std::lock_guard<std::mutex> lock(slots_mutex_);
            camera_snapshot = camera_slots_;
        }
        {
            if (camera_snapshot.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            for (auto& slot : camera_snapshot) {
                if (!slot->running) continue;
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
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
            publishFrameSet(std::move(fs));
        } else {
            spdlog::debug("Frame sync skew too large: {:.1f} ms (max: {:.1f} ms)",
                         skew_ms, max_sync_skew_ms_);
            // Discard oldest frames and retry on next iteration
        }
    }
}

void FrameBroker::publishFrameSet(std::shared_ptr<FrameSet> frame_set) {
    if (!latest_delivery_) {
        emit frameSetReady(std::move(frame_set));
        return;
    }
    std::lock_guard<std::mutex> lock(delivery_mutex_);
    pending_frame_set_ = std::move(frame_set);
    if (delivery_queued_) return;
    delivery_queued_ = true;
    QMetaObject::invokeMethod(this, [this] {
        std::shared_ptr<FrameSet> latest;
        {
            std::lock_guard<std::mutex> delivery_lock(delivery_mutex_);
            latest = std::move(pending_frame_set_);
            delivery_queued_ = false;
        }
        if (running_ && latest) emit frameSetReady(std::move(latest));
    }, Qt::QueuedConnection);
}

}  // namespace mocap
