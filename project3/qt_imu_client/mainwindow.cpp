#include "mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFont>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("IMU Orientation Visualizer – Task 3");
    resize(700, 620);

    // ── Worker thread setup ───────────────────────────────────────────────
    m_worker = new ImuWorker;
    m_thread = new QThread(this);
    m_worker->moveToThread(m_thread);

    // Cross-thread signal/slot connections (Qt::QueuedConnection is automatic
    // because emitter and receiver live in different threads)
    connect(this,     &MainWindow::requestConnect,
            m_worker, &ImuWorker::connectToServer);
    connect(this,     &MainWindow::requestDisconnect,
            m_worker, &ImuWorker::disconnectFromServer);

    connect(m_worker, &ImuWorker::quaternionReceived,
            this,     &MainWindow::onQuaternionReceived);
    connect(m_worker, &ImuWorker::statusChanged,
            this,     &MainWindow::onStatusChanged);
    connect(m_worker, &ImuWorker::connectionEstablished,
            this,     &MainWindow::onConnectionEstablished);
    connect(m_worker, &ImuWorker::connectionLost,
            this,     &MainWindow::onConnectionLost);

    // Clean up worker when thread finishes
    connect(m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    m_thread->start();

    // ── Widgets ───────────────────────────────────────────────────────────
    m_gl = new GlWidget;

    m_hostEdit = new QLineEdit("192.168.1.100");
    m_hostEdit->setPlaceholderText("RPi IP address");

    m_portEdit = new QLineEdit("8080");
    m_portEdit->setFixedWidth(70);

    m_connectBtn = new QPushButton("Connect");
    m_connectBtn->setFixedWidth(100);

    m_statusLabel = new QLabel("Disconnected");
    m_statusLabel->setStyleSheet("color:#ff6060;");

    m_quatLabel = new QLabel("q = [1.000000, 0.000000, 0.000000, 0.000000]");
    m_quatLabel->setFont(QFont("Monospace", 10));

    // ── Layout ────────────────────────────────────────────────────────────
    auto* connRow = new QHBoxLayout;
    connRow->addWidget(new QLabel("Host:"));
    connRow->addWidget(m_hostEdit, 1);
    connRow->addWidget(new QLabel("Port:"));
    connRow->addWidget(m_portEdit);
    connRow->addWidget(m_connectBtn);

    auto* infoRow = new QHBoxLayout;
    infoRow->addWidget(new QLabel("Status:"));
    infoRow->addWidget(m_statusLabel, 1);

    auto* ctrlBox = new QGroupBox("Connection");
    auto* ctrlLayout = new QVBoxLayout;
    ctrlLayout->addLayout(connRow);
    ctrlLayout->addLayout(infoRow);
    ctrlLayout->addWidget(m_quatLabel);
    ctrlBox->setLayout(ctrlLayout);

    auto* root = new QVBoxLayout;
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(6);
    root->addWidget(m_gl, 1);
    root->addWidget(ctrlBox);

    auto* central = new QWidget;
    central->setLayout(root);
    setCentralWidget(central);

    // ── Connect button ────────────────────────────────────────────────────
    connect(m_connectBtn, &QPushButton::clicked, this, &MainWindow::onConnectClicked);
}

MainWindow::~MainWindow() {
    emit requestDisconnect();
    m_thread->quit();
    m_thread->wait(2000);
}

// ── Slots ─────────────────────────────────────────────────────────────────────

void MainWindow::onConnectClicked() {
    if (!m_connected) {
        QString host = m_hostEdit->text().trimmed();
        quint16 port = static_cast<quint16>(m_portEdit->text().toUInt());
        emit requestConnect(host, port);
        m_connectBtn->setEnabled(false);
    } else {
        emit requestDisconnect();
        m_connectBtn->setEnabled(false);
    }
}

void MainWindow::onStatusChanged(const QString& msg) {
    m_statusLabel->setText(msg);
}

void MainWindow::onConnectionEstablished() {
    m_connected = true;
    m_connectBtn->setText("Disconnect");
    m_connectBtn->setEnabled(true);
    m_statusLabel->setStyleSheet("color:#60ff60;");
}

void MainWindow::onConnectionLost() {
    m_connected = false;
    m_connectBtn->setText("Connect");
    m_connectBtn->setEnabled(true);
    m_statusLabel->setStyleSheet("color:#ff6060;");
}

void MainWindow::onQuaternionReceived(double w, double x, double y, double z) {
    m_gl->updateQuaternion(w, x, y, z);

    m_quatLabel->setText(
        QString("q = [%1, %2, %3, %4]")
            .arg(w, 8, 'f', 6)
            .arg(x, 8, 'f', 6)
            .arg(y, 8, 'f', 6)
            .arg(z, 8, 'f', 6));
}
