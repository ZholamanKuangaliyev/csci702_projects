#pragma once

#include <QObject>
#include <QTcpSocket>
#include <QByteArray>

// ImuWorker lives in a background QThread.
// It manages a QTcpSocket connection to the RPi TCP server and emits
// quaternionReceived() for every complete 32-byte packet received.
class ImuWorker : public QObject {
    Q_OBJECT
public:
    explicit ImuWorker(QObject* parent = nullptr);
    ~ImuWorker() override;

public slots:
    // Call from main thread via signal or QMetaObject::invokeMethod
    void connectToServer(const QString& host, quint16 port);
    void disconnectFromServer();

signals:
    void quaternionReceived(double w, double x, double y, double z);
    void statusChanged(const QString& message);
    void connectionEstablished();
    void connectionLost();

private slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
    void onError(QAbstractSocket::SocketError err);

private:
    QTcpSocket* m_socket{nullptr};
    QByteArray  m_buffer;          // accumulates partial reads
};
