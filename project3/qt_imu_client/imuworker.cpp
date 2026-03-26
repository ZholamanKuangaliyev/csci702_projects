#include "imuworker.h"
#include <cstring>

ImuWorker::ImuWorker(QObject* parent) : QObject(parent) {}

ImuWorker::~ImuWorker() {
    disconnectFromServer();
}

void ImuWorker::connectToServer(const QString& host, quint16 port) {
    if (!m_socket) {
        m_socket = new QTcpSocket(this);
        connect(m_socket, &QTcpSocket::connected,    this, &ImuWorker::onConnected);
        connect(m_socket, &QTcpSocket::disconnected, this, &ImuWorker::onDisconnected);
        connect(m_socket, &QTcpSocket::readyRead,    this, &ImuWorker::onReadyRead);
        connect(m_socket, &QAbstractSocket::errorOccurred,
                this, &ImuWorker::onError);
    }
    m_buffer.clear();
    emit statusChanged("Connecting to " + host + ":" + QString::number(port) + " …");
    m_socket->connectToHost(host, port);
}

void ImuWorker::disconnectFromServer() {
    if (m_socket) {
        m_socket->disconnectFromHost();
        m_socket->deleteLater();
        m_socket = nullptr;
    }
}

void ImuWorker::onConnected() {
    emit statusChanged("Connected");
    emit connectionEstablished();
}

void ImuWorker::onDisconnected() {
    emit statusChanged("Disconnected");
    emit connectionLost();
}

void ImuWorker::onReadyRead() {
    m_buffer.append(m_socket->readAll());

    // Each quaternion packet is 4 × double = 32 bytes
    constexpr int PACKET_SIZE = 4 * static_cast<int>(sizeof(double));

    while (m_buffer.size() >= PACKET_SIZE) {
        double packet[4];
        std::memcpy(packet, m_buffer.constData(), PACKET_SIZE);
        m_buffer.remove(0, PACKET_SIZE);
        emit quaternionReceived(packet[0], packet[1], packet[2], packet[3]);
    }
}

void ImuWorker::onError(QAbstractSocket::SocketError) {
    emit statusChanged("Error: " + m_socket->errorString());
    emit connectionLost();
}
