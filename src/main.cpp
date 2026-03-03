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

    mocap::MainWindow window;
    window.show();

    return app.exec();
}
