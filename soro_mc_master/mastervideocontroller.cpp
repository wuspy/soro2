#include "mastervideocontroller.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"

#include "maincontroller.h"

#define LogTag "MasterVideoController"

namespace Soro {

MasterVideoController::MasterVideoController(const CameraSettingsModel *cameraSettings, const RosNodeList *rosNodeList, QObject *parent) : QObject(parent)
{
    _cameraSettings = cameraSettings;
    _rosNodeList = rosNodeList;

    for (int i = 0; i < cameraSettings->getCameraCount(); i++)
    {
        QUdpSocket *socket = new QUdpSocket(this);

        if (!socket->bind(SORO_NET_FIRST_VIDEO_PORT + i))
        {
            MainController::panic(LogTag, "Cannot bind to UDP port " + QString::number(SORO_NET_FIRST_VIDEO_PORT + i));
        }
        connect(socket, &QUdpSocket::readyRead, this, [this, socket, i]()
        {
            this->onSocketReadyRead(socket, SORO_NET_MC_FIRST_VIDEO_PORT + i);
        });
        _videoSockets.append(socket);
        Logger::logInfo(LogTag, "Bound UDP video socket " + QString::number(socket->localPort()));
    }

    _audioSocket = new QUdpSocket(this);
    if (!_audioSocket->bind(SORO_NET_AUDIO_PORT))
    {
        MainController::panic(LogTag, "Cannot bind to UDP port " + QString::number(SORO_NET_AUDIO_PORT));
    }

    connect(_audioSocket, &QUdpSocket::readyRead, this, [this]()
    {
        this->onSocketReadyRead(_audioSocket, SORO_NET_MC_AUDIO_PORT);
    });
    Logger::logInfo(LogTag, "Bound UDP audio socket " + QString::number(_audioSocket->localPort()));

    connect(_rosNodeList, &RosNodeList::nodesUpdated, this, &MasterVideoController::onRosNodeListUpdated);

    _announceTimerId = startTimer(1000);
}

void MasterVideoController::onRosNodeListUpdated()
{
    _bounceAddresses.clear();
    for (RosNodeList::Node node : _rosNodeList->getNodes())
    {
        if (node.name.startsWith("/mc_") && node.name != "/mc_master")
        {
            _bounceAddresses.append(node.address);
        }
    }
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

} // namespace Soro
