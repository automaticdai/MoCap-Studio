#pragma once

#include "core/types.h"
#include "core/config.h"
#include "core/camera_intrinsics.h"

namespace mocap {

class ICameraSource {
public:
    virtual ~ICameraSource() = default;

    virtual bool open(const CameraConfig& config) = 0;
    virtual void close() = 0;
    virtual bool isOpened() const = 0;

    // Live sources use steady_clock time_since_epoch seconds so independently
    // opened cameras share the same timestamp origin. Files use media time.
    virtual bool grabFrame(CapturedFrame& out, int timeout_ms = 100) = 0;
    virtual bool prefersLatestFrame() const { return false; }
    virtual bool isLive() const { return true; }

    virtual CameraIntrinsics intrinsics() const = 0;
    virtual std::string id() const = 0;
    virtual std::string displayName() const = 0;
};

}  // namespace mocap
