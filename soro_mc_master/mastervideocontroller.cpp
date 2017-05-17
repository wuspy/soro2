#include "mastervideocontroller.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"
#include "soro_core/addmediabouncemessage.h"

#include "maincontroller.h"

#define LogTag "MasterVideoController"

namespace Soro {

MasterVideoController::MasterVideoController(const SettingsModel *settings, const CameraSettingsModel *cameraSettings, QObject *parent) : QObject(parent)
{
    _cameraSettings = cameraSettings;
    _settings = settings;

    for (int i = 0; i < cameraSettings->getCameraCount(); i++)
    {
        QUdpSocket *socket = new QUdpSocket(this);

        if (!socket->bind())
        {
            MainController::panic(LogTag, "Cannot bind UDP socket");
        }
        if (!socket->open(QIODevice::ReadWrite))
        {
            MainController::panic(LogTag, "Cannot open UDP socket");
        }
        connect(socket, &QUdpSocket::readyRead, this, [this, socket, i]()
        {
            this->onSocketReadyRead(socket, SORO_NET_MC_FIRST_VIDEO_PORT + i);
        });
        _videoSockets.append(socket);
        LOG_I(LogTag, "Bound UDP video socket");
    }

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    connect(_mqtt, &QMQTT::Client::received, this, &MasterVideoController::onMqttMessage);
    connect(_mqtt, &QMQTT::Client::connected, this, &MasterVideoController::onMqttConnected);
    connect(_mqtt, &QMQTT::Client::disconnected, this, &MasterVideoController::onMqttDisconnected);
    _mqtt->setClientId("master_video_controller");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(1);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    _announceTimerId = startTimer(1000);
}

void MasterVideoController::onMqttMessage(const QMQTT::Message &msg)
{
    if (msg.topic() == "video_bounce")
    {
        AddMediaBounceMessage bounceMsg(msg.payload());
        if (bounceMsg.address == QHostAddress::Null)
        {
            LOG_W(LogTag, "Got video_bounce message with invalid address");
        }
        else
        {
            if (!_bounceMap.contains(bounceMsg.clientID))
            {
                LOG_I(LogTag, "Adding client " + bounceMsg.clientID + " at " + bounceMsg.address.toString() + " to video bounce list");
                _bounceMap.insert(bounceMsg.clientID, bounceMsg.address);
                _bounceAddresses = _bounceMap.values();
                Q_EMIT bounceAddressesChanged(_bounceMap);
            }
        }
    }
    else if (msg.topic() == "system_down")
    {
        QString clientID(msg.payload());

        if (_bounceMap.contains(clientID))
        {
            LOG_I(LogTag, "Removing client " + clientID + " at " + _bounceMap.value(clientID).toString() + " from video bounce list");
            _bounceMap.remove(clientID);
            _bounceAddresses = _bounceMap.values();
            Q_EMIT bounceAddressesChanged(_bounceMap);
        }
    }
}

void MasterVideoController::onMqttConnected()
{
    LOG_I(LogTag, "Connected to MQTT broker");
    _mqtt->subscribe("video_bounce", 0);
    _mqtt->subscribe("system_down", 1);
}

void MasterVideoController::onMqttDisconnected()
{
    LOG_W(LogTag, "Disconnected from MQTT broker");
}

void MasterVideoController::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _announceTimerId)
    {
        for (int i = 0; i < _videoSockets.size(); ++i)
        {
            _videoSockets[i]->writeDatagram("video", 6, _settings->getMqttBrokerAddress(), SORO_NET_FIRST_VIDEO_PORT + i);
        }
    }
}

void MasterVideoController::onSocketReadyRead(QUdpSocket *socket, quint16 bouncePort)
{
    quint32 totalLen = 0;
    while (socket->hasPendingDatagrams())
    {
        qint64 len = socket->readDatagram(_buffer, 65536);
        for (QHostAddress bounceAddress : _bounceAddresses)
        {
            socket->writeDatagram(_buffer, len, bounceAddress, bouncePort);
        }
        totalLen += len;
    }
    Q_EMIT bytesDown(totalLen);
}

} // namespace Soro
