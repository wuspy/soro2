#include "audioserver.h"
#include "soro_core/constants.h"
#include "soro_core/logger.h"
#include "soro_core/gstreamerutil.h"
#include "soro_core/notificationmessage.h"

#include "maincontroller.h"

#define LogTag "AudioServer"

namespace Soro {

AudioServer::AudioServer(const SettingsModel *settings, QObject *parent) : QObject(parent)
{
    _settings = settings;
    _hasWaitingAssignment = false;
    _hasCurrentAssignment = false;
    _child = nullptr;
    _childInterface = nullptr;

    // Ensure child process executable exits
    if (!QFile(SORO_ROVER_AUDIO_STREAM_PROCESS_PATH).exists())
    {
        MainController::panic(LogTag, "Audio stream process is not at the correct path");
    }

    LOG_I(LogTag, "Registering as D-Bus RPC object...");
    // Register this class as a D-Bus RPC service so other processes can call our public slots
    QDBusConnection::sessionBus().registerObject("/", this, QDBusConnection::ExportAllSlots);

    connect(&_audioSocket, &QUdpSocket::readyRead, this, [this]()
    {
        char buffer[6];
        QHostAddress host;
        quint16 port;
        while (_audioSocket.hasPendingDatagrams())
        {
            _audioSocket.readDatagram(buffer, 6, &host, &port);
            if (strncmp(buffer, "audio", 5) == 0)
            {
                // Valid message
                if ((_clientAddress != host) || (_clientPort != port))
                {
                    // Address changed, see if there is a stream running
                    if (_hasWaitingAssignment)
                    {
                        // Modify the waiting assignment to point torwards the new address
                        LOG_W(LogTag, "Client has changed addresses while a stream is queued, modifying the queued stream");
                        _waitingAssignment.address = host;
                        _waitingAssignment.port = port;
                    }
                    if (_hasCurrentAssignment)
                    {
                        // Stop the active assignment, and start a new one
                        LOG_W(LogTag, "Client has changed addresses while a stream is in progress, attempting to restart");
                        _currentAssignment.address = host;
                        _currentAssignment.port = port;
                        giveChildAssignment(_currentAssignment);
                    }
                    _clientAddress = host;
                    _clientPort = port;
                }
            }
        }
    });

    if (!_audioSocket.bind(SORO_NET_AUDIO_PORT, QAbstractSocket::ShareAddress))
    {
        MainController::panic(LogTag, "Cannot bind to UDP port");
    }
    if (!_audioSocket.open(QIODevice::ReadOnly))
    {
        MainController::panic(LogTag, "Cannot open UDP port");
    }

    // Start heartbeat timer so children still know we're running
    _heartbeatTimerId = startTimer(1000);

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    connect(_mqtt, &QMQTT::Client::received, this, &AudioServer::onMqttMessage);
    connect(_mqtt, &QMQTT::Client::connected, this, &AudioServer::onMqttConnected);
    connect(_mqtt, &QMQTT::Client::disconnected, this, &AudioServer::onMqttDisconnected);
    _mqtt->setClientId("audio_server");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(1);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();
}

void AudioServer::onMqttConnected()
{
    LOG_I(LogTag, "Connected to MQTT broker");
    _mqtt->subscribe("audio_request", 0);
    Q_EMIT mqttConnected();
}

void AudioServer::onMqttDisconnected()
{
    LOG_W(LogTag, "Disconnected from MQTT broker");
    Q_EMIT mqttDisconnected();
}

void AudioServer::onMqttMessage(const QMQTT::Message &msg)
{
    if (msg.topic() == "audio_request")
    {
        AudioMessage audioMsg(msg.payload());

        LOG_I(LogTag, "Received new audio request");

        if (_clientAddress == QHostAddress::Null)
        {
            // No handshake has been received for this video port
            LOG_E(LogTag, "No destination address available for audio, cannot stream");
            NotificationMessage notifyMsg;
            notifyMsg.level = NotificationMessage::Level_Error;
            notifyMsg.title = "Cannot stream audio";
            notifyMsg.message = "Audio client has not yet given this server an address for this stream";
            _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));
            return;
        }

        Assignment assignment;
        assignment.profile = audioMsg.profile;
        assignment.address = _clientAddress;
        assignment.port = _clientPort;
        assignment.originalMessage = audioMsg;

        if (!_child)
        {
            // Spawn a new child, and queue this assignment to be executed when the child is ready
            LOG_I(LogTag, "Spawning new child...");
            _waitingAssignment = assignment;

            // The first argument to the process, representing the child's name, is the video device
            // it is assigned to work on
            _child->start(SORO_ROVER_AUDIO_STREAM_PROCESS_PATH);

            connect(_child, static_cast<void (QProcess::*)(int)>(&QProcess::finished), this, [this, assignment](int exitCode)
            {
                LOG_W(LogTag, "Child has exited with code " + QString::number(exitCode));

                if (_childInterface)
                {
                    delete _childInterface;
                    _childInterface = nullptr;
                }
                if (_child)
                {
                    delete _child;
                    _child = nullptr;
                }

                if (_hasWaitingAssignment)
                {
                    _hasWaitingAssignment = false;

                    NotificationMessage notifyMsg;
                    notifyMsg.level = NotificationMessage::Level_Error;
                    notifyMsg.title = "Cannot stream audio";
                    notifyMsg.message = "Unexpected error - child process died before accepting its stream assignment.";
                    _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));

                    reportAudioState();
                }
                if (_hasCurrentAssignment)
                {
                    _hasCurrentAssignment = false;

                    NotificationMessage notifyMsg;
                    notifyMsg.level = NotificationMessage::Level_Error;
                    notifyMsg.title = "Error streaming audio";
                    notifyMsg.message = "Unexpected error while streaming this audio. Try agian.";
                    _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));

                    reportAudioState();
                }
            });
        }
        else
        {
            _hasWaitingAssignment = false;
            _hasCurrentAssignment = false;

            if (_childInterface && _childInterface->isValid())
            {
                // Child is running and can accept assignments
                giveChildAssignment(assignment);
                _currentAssignment = assignment;
                _hasCurrentAssignment = true;
                reportAudioState();
            }
            else
            {
                // Process exists, however it is still starting up and this assignment
                // must be queued
                _waitingAssignment = assignment;
                _hasWaitingAssignment = true;
            }
        }
    }
}

AudioServer::Assignment::Assignment()
{
    address = QHostAddress::Null;
    port = 0;
    profile = GStreamerUtil::AudioProfile();
    originalMessage = AudioMessage();
}

AudioServer::~AudioServer()
{
    // Tell all children to stop themselves
    terminateChild();
}

void AudioServer::terminateChild()
{
    if (_child)
    {
        if (_child->state() == QProcess::Running)
        {
            _child->terminate();
            if (!_child->waitForFinished(1000))
            {
                _child->kill();
                _child->waitForFinished(3000);
            }
        }

        delete _child;
        _child = nullptr;
    }

    if (_childInterface)
    {
        delete _childInterface;
        _childInterface = nullptr;
    }
}

void AudioServer::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _heartbeatTimerId)
    {
        if (_childInterface && _childInterface->isValid())
        {
            _childInterface->call(QDBus::NoBlock, "heartbeat");
        }
    }
}

void AudioServer::giveChildAssignment(Assignment assignment)
{
    if (_childInterface && _childInterface->isValid())
    {
        if (assignment.profile.codec == GStreamerUtil::CODEC_NULL)
        {
            _childInterface->call(
                        QDBus::NoBlock,
                        "stop");

        }
        else
        {
            _childInterface->call(
                        QDBus::NoBlock,
                        "stream",
                        assignment.address.toString(),
                        assignment.port,
                        assignment.profile.toString());
        }
    }
    else
    {
        LOG_E(LogTag, "Tried to give child assignment with invalid dbus interface");
    }
}

void AudioServer::onChildLogInfo(const QString &tag, const QString &message)
{
    LOG_I(QString("[child] %1").arg(tag), message);
}

void AudioServer::onChildReady()
{
    LOG_I(LogTag, "Child is ready to accept an assignment");

    if (!_childInterface)
    {
        // Open a D-Bus interface to this child
        LOG_I(LogTag, "Opening D-Bus interface to child");
        _childInterface = new QDBusInterface(
                                    SORO_DBUS_AUDIO_CHILD_SERVICE_NAME,
                                    "/",
                                    "",
                                    QDBusConnection::sessionBus(),
                                    this);
    }

    if (!_childInterface->isValid())
    {
        LOG_E(LogTag, "Cannot create D-Bus connection to child even though it has a D-Bus conneciton to us");
        terminateChild();

        if (_hasWaitingAssignment)
        {
            NotificationMessage notifyMsg;
            notifyMsg.level = NotificationMessage::Level_Error;
            notifyMsg.title = "Cannot stream audio";
            notifyMsg.message = "Unexpected error - cannot create D-Bus connection to child stream process";
            _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));
            _waitingAssignment = Assignment();
            _hasWaitingAssignment = false;
        }

        reportAudioState();
        return;
    }

    _currentAssignment = Assignment();

    // Check if there is a stream assignment waiting for this child
    if (_hasWaitingAssignment)
    {
        giveChildAssignment(_waitingAssignment);
        _currentAssignment = _waitingAssignment;
        _hasCurrentAssignment = true;
        _hasWaitingAssignment = false;
    }

    reportAudioState();
}

void AudioServer::reportAudioState()
{
    if (_hasCurrentAssignment)
    {
        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "audio_state", _currentAssignment.originalMessage, 0, true)); // <-- Retain message
    }
    else
    {
       _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "audio_state", AudioMessage(), 0, true)); // <-- Retain message
    }
}

void AudioServer::onChildError(QString message)
{
    LOG_E(LogTag, "Child reports an error: " + message);
    if (_hasCurrentAssignment)
    {
        // Send a message on the notification topic
        NotificationMessage notifyMsg;
        notifyMsg.level = NotificationMessage::Level_Error;
        notifyMsg.title = "Error streaming audio";
        notifyMsg.message = "Stream error: " + message;

        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));
        _hasCurrentAssignment = false;
        reportAudioState();
    }
}

void AudioServer::onChildStreaming()
{
    LOG_I(LogTag, "Child has started streaming");
}

} // namespace Soro
