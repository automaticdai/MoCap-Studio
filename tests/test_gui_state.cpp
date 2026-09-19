#include <gtest/gtest.h>
#include <QApplication>
#include <QDockWidget>
#include <QTemporaryDir>
#include <QSettings>
#include <QLineEdit>
#include <QTimer>
#include "gui/camera_feed_widget.h"
#include "gui/main_window.h"
#include "gui/mocap_canvas.h"
#include "gui/session_dialog.h"
#include "gui/timeline_widget.h"
#include "storage/session_manager.h"
#include <fstream>
#include <thread>
#include <cstdlib>

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    QTemporaryDir settings;
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, settings.path());
    return RUN_ALL_TESTS();
}

namespace {
std::shared_ptr<mocap::FrameSet> sampleFrame() {
    auto frames = std::make_shared<mocap::FrameSet>();
    mocap::CapturedFrame frame;
    frame.camera_id = "cam0";
    frame.image = cv::Mat(1080, 1920, CV_8UC3, cv::Scalar(40, 100, 180));
    frames->frames.push_back(frame);
    return frames;
}
}

TEST(CameraFeedUi, RepeatedFramesDoNotGrowDockAndRemovedCamerasDisappear) {
    QMainWindow window;
    auto* dock = new QDockWidget("Camera Feeds", &window);
    auto* feeds = new mocap::CameraFeedWidget;
    dock->setWidget(feeds);
    window.addDockWidget(Qt::LeftDockWidgetArea, dock);
    window.setCentralWidget(new QWidget);
    window.resize(1000, 650);
    feeds->setCameras({"cam0"});
    window.show();
    QApplication::processEvents();
    const auto before = dock->size();
    const auto outer = window.size();
    auto frame = sampleFrame();
    for (int i = 0; i < 150; ++i) {
        feeds->onFrameSet(frame);
        QApplication::processEvents();
    }
    EXPECT_EQ(dock->size(), before);
    EXPECT_EQ(window.size(), outer);
    EXPECT_NE(feeds->findChild<QLabel*>("cameraPreview_cam0"), nullptr);
    feeds->setCameras({"cam1"});
    EXPECT_EQ(feeds->findChild<QLabel*>("cameraPreview_cam0"), nullptr);
    ASSERT_NE(feeds->findChild<QLabel*>("cameraPreview_cam1"), nullptr);
    feeds->setCameraStatus("cam1", "Connection failed");
    EXPECT_TRUE(feeds->findChild<QLabel*>("cameraPreview_cam1")->text().contains("Connection failed"));
}

TEST(SessionUi, NewSessionUpdatesNameResetsTimelineAndPersistsMetadata) {
    QTemporaryDir temporary;
    const auto config = temporary.filePath("config.yaml");
    std::ofstream(config.toStdString()) << "cameras: []\n";
    mocap::MainWindow window(config.toStdString());
    auto* timeline = window.findChild<mocap::TimelineWidget*>();
    ASSERT_NE(timeline, nullptr);
    timeline->setDuration(123);
    timeline->setCurrentTime(99);
    QTimer::singleShot(0, [&] {
        auto* dialog = qobject_cast<mocap::SessionDialog*>(QApplication::activeModalWidget());
        ASSERT_NE(dialog, nullptr);
        const auto fields = dialog->findChildren<QLineEdit*>();
        ASSERT_GE(fields.size(), 2);
        fields[0]->setText("Walking test");
        fields[1]->setText(temporary.path());
        dialog->accept();
    });
    ASSERT_TRUE(QMetaObject::invokeMethod(&window, "onNewSession", Qt::DirectConnection));
    EXPECT_TRUE(window.windowTitle().contains("Walking test"));
    auto* status = window.findChild<QLabel*>("sessionStatus");
    ASSERT_NE(status, nullptr);
    EXPECT_TRUE(status->text().contains("Walking test"));
    EXPECT_DOUBLE_EQ(timeline->currentTime(), 0);
    EXPECT_DOUBLE_EQ(timeline->duration(), 0);
    mocap::SessionManager stored;
    ASSERT_TRUE(stored.openSession(status->toolTip().toStdString()));
    EXPECT_EQ(stored.metadata().name, "Walking test");
    auto* reconstruction = window.findChild<QLabel*>("reconstructionStatus");
    ASSERT_NE(reconstruction, nullptr);
    EXPECT_TRUE(reconstruction->text().contains("0 connected"));
}

TEST(SessionStorage, CreatingAnotherSessionDoesNotOverwriteThePreviousOne) {
    QTemporaryDir temporary;
    mocap::SessionManager sessions;
    const auto first = sessions.createSession(temporary.path().toStdString(), 30, {}, "First");
    const auto second = sessions.createSession(temporary.path().toStdString(), 30, {}, "Second");
    EXPECT_NE(first, second);
    ASSERT_TRUE(sessions.openSession(first));
    EXPECT_EQ(sessions.metadata().name, "First");
}

TEST(CanvasUi, DrawsGridAndSynthetic3dJoints) {
    if (!std::getenv("MOCAP_TEST_GL")) GTEST_SKIP() << "Needs an OpenGL display";
    mocap::MoCapCanvas canvas;
    canvas.setStatusMessage({});
    canvas.resize(640, 480);
    canvas.show();
    for (int i = 0; i < 10; ++i) {
        QApplication::processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(canvas.isValid());
    const auto grid = canvas.grabFramebuffer();
    ASSERT_FALSE(grid.isNull());
    const auto background = grid.pixelColor(0, 0);
    int grid_pixels = 0;
    for (int y = 0; y < grid.height(); ++y)
        for (int x = 0; x < grid.width(); ++x)
            grid_pixels += grid.pixelColor(x, y) != background;
    EXPECT_GT(grid_pixels, 1000);
    mocap::Pose3D pose;
    for (int index : {1, 2, 5, 8}) {
        mocap::Marker3D marker;
        marker.index = index;
        marker.confidence = 1;
        marker.position = mocap::Vec3f(index == 2 ? -0.5f : index == 5 ? 0.5f : 0,
                                      index == 8 ? 0.5f : 1.5f, 0);
        pose.markers.push_back(marker);
    }
    canvas.onPose3DUpdate({pose});
    QApplication::processEvents();
    const auto skeleton = canvas.grabFramebuffer();
    int green_pixels = 0;
    for (int y = 0; y < skeleton.height(); ++y)
        for (int x = 0; x < skeleton.width(); ++x) {
            const auto pixel = skeleton.pixelColor(x, y);
            green_pixels += pixel.green() > 150 && pixel.red() < 30 && pixel.blue() > 80;
        }
    EXPECT_GT(green_pixels, 50);
    if (const char* path = std::getenv("MOCAP_TEST_SCREENSHOT")) skeleton.save(path);
}

TEST(LiveUi, CameraPreviewAppearsWithoutCreatingSession) {
    const char* url = std::getenv("MOCAP_TEST_UI_RTSP");
    if (!url) GTEST_SKIP() << "Needs a live RTSP camera and a display";
    QTemporaryDir temporary;
    mocap::AppConfig config;
    mocap::CameraConfig camera;
    camera.id = "cam0";
    camera.type = "ip";
    camera.url = url;
    config.cameras.push_back(camera);
    const auto config_path = temporary.filePath("config.yaml").toStdString();
    config.save(config_path);
    mocap::MainWindow window(config_path);
    window.show();
    auto* preview = window.findChild<QLabel*>("cameraPreview_cam0");
    ASSERT_NE(preview, nullptr);
    for (int i = 0; i < 200 && !preview->text().isEmpty(); ++i) {
        QApplication::processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_TRUE(preview->text().isEmpty()) << preview->text().toStdString();
    EXPECT_TRUE(window.findChild<QLabel*>("sessionStatus")->text().startsWith("No session"));
    EXPECT_TRUE(window.findChild<QLabel*>("reconstructionStatus")->text().contains("1 connected"));
    const auto size = window.size();
    for (int i = 0; i < 60; ++i) {
        QApplication::processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_EQ(window.size(), size);
    if (const char* path = std::getenv("MOCAP_TEST_UI_SCREENSHOT")) window.grab().save(path);
}
