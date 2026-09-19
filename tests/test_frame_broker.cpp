#include <gtest/gtest.h>
#include <QCoreApplication>
#include "capture/frame_broker.h"
#include <chrono>
#include <thread>

namespace {
class FastCamera : public mocap::ICameraSource {
public:
    std::string camera_id = "fake";
    bool live = true;
    std::atomic<int> produced{0};
    bool open(const mocap::CameraConfig&) override { return true; }
    void close() override {}
    bool isOpened() const override { return true; }
    bool prefersLatestFrame() const override { return true; }
    bool isLive() const override { return live; }
    bool grabFrame(mocap::CapturedFrame& frame, int) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        frame.camera_id = id();
        frame.frame_number = ++produced;
        frame.timestamp = frame.frame_number * 0.002;
        return true;
    }
    mocap::CameraIntrinsics intrinsics() const override { return {}; }
    std::string id() const override { return camera_id; }
    std::string displayName() const override { return id(); }
};

void ensureApplication() {
    static int argc = 1;
    static char name[] = "test_frame_broker";
    static char* argv[] = {name, nullptr};
    static QCoreApplication app(argc, argv);
}

TEST(FrameBroker, RemovingFirstCameraKeepsRemainingCameraValid) {
    ensureApplication();
    mocap::FrameBroker broker;
    auto first = std::make_shared<FastCamera>();
    auto second = std::make_shared<FastCamera>();
    second->camera_id = "second";
    broker.addCamera(first);
    broker.addCamera(second);
    broker.removeCamera(first->id());
    EXPECT_EQ(broker.cameraCount(), 1);
    broker.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    broker.stop();
    EXPECT_GT(second->produced, 0);
    EXPECT_EQ(first->produced, 0);
}

TEST(FrameBroker, IdlePreviewDoesNotConsumeVideoFiles) {
    ensureApplication();
    mocap::FrameBroker broker;
    auto file = std::make_shared<FastCamera>();
    file->live = false;
    broker.addCamera(file);
    broker.start(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    broker.stop();
    EXPECT_EQ(file->produced, 0);
    broker.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    broker.stop();
    EXPECT_GT(file->produced, 0);
}
}

TEST(FrameBroker, SlowGuiGetsLatestAndStopDoesNotDeadlock) {
    ensureApplication();
    mocap::FrameBroker broker;
    auto camera = std::make_shared<FastCamera>();
    broker.addCamera(camera);
    int received = 0;
    int last_frame = 0;
    QObject::connect(&broker, &mocap::FrameBroker::frameSetReady, &broker,
        [&](std::shared_ptr<mocap::FrameSet> frames) {
            ++received;
            last_frame = frames->frames.front().frame_number;
        });
    broker.start();
    // Deliberately do not service the GUI event loop while capture continues.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    const int before = camera->produced;
    ASSERT_GT(before, 10);
    QCoreApplication::processEvents();
    EXPECT_GE(received, 1);
    EXPECT_LE(received, 2);
    EXPECT_GE(last_frame, before - 5);
    const auto start = std::chrono::steady_clock::now();
    broker.stop();
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(1));
    int stopped_count = received;
    QCoreApplication::processEvents();
    EXPECT_EQ(received, stopped_count);

    broker.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    QCoreApplication::processEvents();
    EXPECT_GT(received, stopped_count);
    broker.stop();
}
