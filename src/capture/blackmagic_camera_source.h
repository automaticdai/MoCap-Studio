#pragma once

#ifdef MOCAP_HAS_BLACKMAGIC

#include "capture/icamera_source.h"

namespace mocap {

class BlackmagicCameraSource : public ICameraSource {
public:
    BlackmagicCameraSource() = default;
    ~BlackmagicCameraSource() override;

    bool open(const CameraConfig& config) override;
    void close() override;
    bool isOpened() const override;

    bool grabFrame(CapturedFrame& out, int timeout_ms = 100) override;

    CameraIntrinsics intrinsics() const override;
    std::string id() const override;
    std::string displayName() const override;

private:
    CameraConfig config_;
    CameraIntrinsics intrinsics_;
    bool opened_ = false;
};

}  // namespace mocap

#endif  // MOCAP_HAS_BLACKMAGIC
