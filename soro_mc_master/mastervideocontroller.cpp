#include "mastervideocontroller.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"

#define LogTag "MasterVideoController"

namespace Soro {

MasterVideoController::MasterVideoController(const CameraSettingsModel *cameraSettings, QHostAddress serverAddress, QObject *parent) : QObject(parent)
{
    _cameraSettings = cameraSettings;
    _serverAddress = serverAddress;

    for (int i = 0; i < cameraSettings->getCameraCount(); i++)
    {
        QUdpSocket *socket = new QUdpSocket(this);
        socket->bind();
        connect(socket, &QUdpSocket::readyRead, this, [this, socket, i]()
        {
            this->onSocketReadyRead(socket, SORO_NET_FIRST_VIDEO_PORT + i);
        });
        _videoSockets.append(socket);
        Logger::logInfo(LogTag, "Bound UDP video socket " + QString::number(socket->localPort()));
    }


    _audioSocket = new QUdpSocket(this);
    _audioSocket->bind();
    connect(_audioSocket, &QUdpSocket::readyRead, this, [this]()
    {
        this->onSocketReadyRead(_audioSocket, SORO_NET_AUDIO_PORT);
    });
    Logger::logInfo(LogTag, "Bound UDP audio socket " + QString::number(_audioSocket->localPort()));

    _punchTimerId = startTimer(1000);
}

void MasterVideoController::onSocketReadyRead(QUdpSocket *socket, quint16 bouncePort)
{
    quint32 totalLen;
    while (socket->hasPendingDatagrams())
    {
        int len = socket->read(_buffer, 65536);
        for (int i = 0; i < _bounceAddresses.length(); ++i)
        {
            socket->writeDatagram(_buffer, len, _bounceAddresses.value(i), bouncePort);
        }
        totalLen += len;
    }
    Q_EMIT bitsDown(totalLen);
}

void MasterVideoController::addPunchHost(QHostAddress address)
{
    if (!_bounceAddresses.contains(address))
    {
        _bounceAddresses.append(address);
    }
}

void MasterVideoController::removePunchHost(QHostAddress address)
{
    _bounceAddresses.removeAll(address);
}

void MasterVideoController::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _punchTimerId)
    {
        for (int i = 0; i < _videoSockets.size(); i++)
        {
            QString msg = "video" + QString::number(i);
            _videoSockets[i]->writeDatagram(msg.toLatin1().constData(), msg.length() + 1, _serverAddress, SORO_NET_FIRST_VIDEO_PORT + i);
        }
        QString msg = "audio";
        _audioSocket->writeDatagram(msg.toLatin1().constData(), msg.length() + 1, _serverAddress, SORO_NET_AUDIO_PORT);
    }
}

} // namespace Soro
