#include <QApplication>
#include "mainwindow.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("IMU Orientation Visualizer");

    MainWindow w;
    w.show();
    return app.exec();
}
