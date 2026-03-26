#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QThread>
#include "glwidget.h"
#include "imuworker.h"

// Task 3 – Main window for IMU orientation visualizer.
// Main thread: GUI + GlWidget rendering.
// Worker QThread: TCP receive loop (ImuWorker).
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

signals:
    // Forwarded to worker thread (cross-thread safe)
    void requestConnect(const QString& host, quint16 port);
    void requestDisconnect();

private slots:
    void onConnectClicked();
    void onStatusChanged(const QString& msg);
    void onConnectionEstablished();
    void onConnectionLost();
    void onQuaternionReceived(double w, double x, double y, double z);

private:
    GlWidget*    m_gl{nullptr};
    QLineEdit*   m_hostEdit{nullptr};
    QLineEdit*   m_portEdit{nullptr};
    QPushButton* m_connectBtn{nullptr};
    QLabel*      m_statusLabel{nullptr};
    QLabel*      m_quatLabel{nullptr};

    QThread*    m_thread{nullptr};
    ImuWorker*  m_worker{nullptr};

    bool m_connected{false};
};
