#include <QApplication>
#include <spdlog/spdlog.h>
#include "gui/main_window.h"

int main(int argc, char* argv[]) {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("MoCap Studio v{}", "0.1.0");

    QApplication app(argc, argv);
    app.setApplicationName("MoCap Studio");
    app.setApplicationVersion("0.1.0");
    app.setOrganizationName("MoCap Studio");

    // Dark palette
    QPalette palette;
    palette.setColor(QPalette::Window, QColor(42, 42, 48));
    palette.setColor(QPalette::WindowText, QColor(204, 204, 204));
    palette.setColor(QPalette::Base, QColor(30, 30, 36));
    palette.setColor(QPalette::AlternateBase, QColor(34, 34, 42));
    palette.setColor(QPalette::Text, QColor(204, 204, 204));
    palette.setColor(QPalette::Button, QColor(50, 50, 58));
    palette.setColor(QPalette::ButtonText, QColor(204, 204, 204));
    palette.setColor(QPalette::Highlight, QColor(0, 120, 215));
    palette.setColor(QPalette::HighlightedText, QColor(255, 255, 255));
    app.setPalette(palette);

    // Config path from command line or default
    std::string config_path = "config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    mocap::MainWindow window(config_path);
    window.show();

    return app.exec();
}
