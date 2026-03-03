#ifdef MOCAP_HAS_BLACKMAGIC

#include "capture/blackmagic_camera_source.h"
#include <spdlog/spdlog.h>

namespace mocap {

BlackmagicCameraSource::~BlackmagicCameraSource() {
    close();
}

bool BlackmagicCameraSource::open(const CameraConfig& config) {
    config_ = config;
    // TODO: Integrate DeckLink SDK for real Blackmagic hardware support
    spdlog::warn("Blackmagic DeckLink support is a stub — no real hardware integration yet");
    return false;
}

void BlackmagicCameraSource::close() {
    opened_ = false;
}

bool BlackmagicCameraSource::isOpened() const {
    return opened_;
}

bool BlackmagicCameraSource::grabFrame(CapturedFrame& /*out*/, int /*timeout_ms*/) {
    return false;
}

CameraIntrinsics BlackmagicCameraSource::intrinsics() const {
    return intrinsics_;
}

std::string BlackmagicCameraSource::id() const {
    return config_.id;
}

std::string BlackmagicCameraSource::displayName() const {
    return "Blackmagic (" + config_.id + ")";
}

}  // namespace mocap

#endif  // MOCAP_HAS_BLACKMAGIC
