#include "videoserver.h"
#include "soro_core/constants.h"
#include "soro_core/logger.h"
#include "soro_core/gstreamerutil.h"
#include "soro_core/notificationmessage.h"

#include "maincontroller.h"

#define LogTag "VideoServer"

namespace Soro {

VideoServer::VideoServer(const SettingsModel *settings, QObject *parent) : QObject(parent)
{
    _settings = settings;

    // Ensure child process executable exits
    if (!QFile(SORO_ROVER_VIDEO_STREAM_PROCESS_PATH).exists())
    {
        MainController::panic(LogTag, "Video stream process is not at the correct path");
    }

    LOG_I(LogTag, "Registering as D-Bus RPC object...");
    // Register this class as a D-Bus RPC service so other processes can call our public slots
    QDBusConnection::sessionBus().registerObject("/", this, QDBusConnection::ExportAllSlots);

    for (quint16 i = SORO_NET_FIRST_VIDEO_PORT; i < SORO_NET_LAST_VIDEO_PORT; ++i)
    {
        QUdpSocket *socket = new QUdpSocket(this);
        _videoSockets.insert(i, socket);
        connect(socket, &QUdpSocket::readyRead, this, [this, socket, i]()
        {
            char buffer[6];
            QHostAddress host;
            quint16 port;
            while (socket->hasPendingDatagrams())
            {
                socket->readDatagram(buffer, 6, &host, &port);
                if (strncmp(buffer, "video", 5) == 0)
                {
                    // Valid message
                    if (_clientAddresses.contains(i))
                    {
                        if ((_clientAddresses.value(i) != host) || (_clientPorts.value(i) != port))
                        {
                            // Address has changed, see if there is a stream running
                            for (QString device : _waitingAssignments.keys())
                            {
                                if (_waitingAssignments[device].originalMessage.camera_index == i - SORO_NET_FIRST_VIDEO_PORT)
                                {
                                    // Modify this waiting assignment to point torwards the new address
                                    LOG_W(LogTag, "Client has changed addresses while a stream is queued, modifying the queued stream");
                                    _waitingAssignments[device].address = host;
                                    _waitingAssignments[device].port = port;
                                    break;
                                }
                            }
                            for (QString device : _currentAssignments.keys())
                            {
                                if (_currentAssignments[device].originalMessage.camera_index == i - SORO_NET_FIRST_VIDEO_PORT)
                                {
                                    // Stop this active assignment, and start a new one
                                    LOG_W(LogTag, "Client has changed addresses while a stream is in progress, attempting to restart");
                                    _currentAssignments[device].address = host;
                                    _currentAssignments[device].port = port;
                                    giveChildAssignment(_currentAssignments[device]);
                                    break;
                                }
                            }
                        }
                    }
                    _clientAddresses.insert(i, host);
                    _clientPorts.insert(i, port);
                }
            }
        });

        if (!socket->bind(i, QAbstractSocket::ShareAddress))
        {
            MainController::panic(LogTag, "Cannot bind to UDP port " + QString::number(i));
        }
        if (!socket->open(QIODevice::ReadOnly))
        {
            MainController::panic(LogTag, "Cannot open UDP port " + QString::number(i));
        }
    }

    // Populate VAAPI option map
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_H264, settings->getUseH264Vaapi());
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_H265, settings->getUseH265Vaapi());
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_VP8, settings->getUseVP8Vaapi());
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_VP9, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_MPEG2, settings->getUseMpeg2Vaapi());
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_MJPEG, settings->getUseJpegVaapi());
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_MPEG4, false);

    // Start heartbeat timer so children still know we're running
    _heartbeatTimerId = startTimer(1000);

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    connect(_mqtt, &QMQTT::Client::received, this, &VideoServer::onMqttMessage);
    connect(_mqtt, &QMQTT::Client::connected, this, &VideoServer::onMqttConnected);
    connect(_mqtt, &QMQTT::Client::disconnected, this, &VideoServer::onMqttDisconnected);
    _mqtt->setClientId("videoserver_" + QString::number(settings->getComputerIndex()));
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(1);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();
}

void VideoServer::onMqttConnected()
{
    LOG_I(LogTag, "Connected to MQTT broker");
    _mqtt->subscribe("video_request", 0);
    _mqtt->subscribe("video_state", 0);
    Q_EMIT mqttConnected();
}

void VideoServer::onMqttDisconnected()
{
    LOG_W(LogTag, "Disconnected from MQTT broker");
    Q_EMIT mqttDisconnected();
}

void VideoServer::onMqttMessage(const QMQTT::Message &msg)
{
    if (msg.topic() == "video_request")
    {
        VideoMessage videoMsg(msg.payload());

        if (videoMsg.camera_computerIndex == _settings->getComputerIndex())
        {
            LOG_I(LogTag, "Received new video request for this server");

            if (!_clientAddresses.contains(SORO_NET_FIRST_VIDEO_PORT + videoMsg.camera_index))
            {
                // No handshake has been received for this video port
                LOG_E(LogTag, "No destination address available for this video port, cannot stream");
                NotificationMessage notifyMsg;
                notifyMsg.level = NotificationMessage::Level_Error;
                notifyMsg.title = "Cannot stream " + videoMsg.camera_name;
                notifyMsg.message = "Video client has not yet given this server an address for this stream";
                _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));
                return;
            }

            QString cameraDevice = findUsbCamera(videoMsg.camera_serial,
                                                 videoMsg.camera_productId,
                                                 videoMsg.camera_vendorId,
                                                 videoMsg.camera_offset);
            if (cameraDevice.isEmpty())
            {
                // This camera wasn't found on our computer. Notify mission control
                LOG_E(LogTag, "Requested camera definition does not exist on this computer");
                NotificationMessage notifyMsg;
                notifyMsg.level = NotificationMessage::Level_Error;
                notifyMsg.title = "Cannot stream " + videoMsg.camera_name;
                notifyMsg.message = "The definition for this camera does not match any cameras connected to this server.";
                _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));
                return;
            }

            Assignment assignment;
            assignment.device = cameraDevice;
            assignment.cameraName = videoMsg.camera_name;
            assignment.vaapi = _useVaapi.value(videoMsg.profile.codec);
            assignment.profile = videoMsg.profile;
            assignment.isStereo = videoMsg.isStereo;
            assignment.address = _clientAddresses.value(SORO_NET_FIRST_VIDEO_PORT + videoMsg.camera_index);
            assignment.port = _clientPorts.value(SORO_NET_FIRST_VIDEO_PORT + videoMsg.camera_index);
            assignment.originalMessage = videoMsg;

            if (videoMsg.isStereo)
            {
                QString cameraDevice2 = findUsbCamera(videoMsg.camera_serial2,
                                                     videoMsg.camera_productId2,
                                                     videoMsg.camera_vendorId2,
                                                     videoMsg.camera_offset2);
                if (cameraDevice2.isEmpty())
                {
                    // This camera wasn't found on our computer. Notify mission control
                    LOG_E(LogTag, "Requested camera definition does not exist on this computer");
                    NotificationMessage notifyMsg;
                    notifyMsg.level = NotificationMessage::Level_Error;
                    notifyMsg.title = "Cannot stream " + videoMsg.camera_name;
                    notifyMsg.message = "The definition for this camera (right channel) does not match any cameras connected to this server.";
                    _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));
                    return;
                }

                assignment.device2 = cameraDevice2;
            }

            if (!_children.contains(assignment.device))
            {
                // Spawn a new child, and queue this assignment to be executed when the child is ready
                LOG_I(LogTag, "Spawning new child for " + assignment.device + "...");
                QProcess *child = new QProcess(this);
                _children.insert(assignment.device, child);
                _waitingAssignments.insert(assignment.device, assignment);

                // The first argument to the process, representing the child's name, is the video device
                // it is assigned to work on
                child->start(SORO_ROVER_VIDEO_STREAM_PROCESS_PATH, QStringList() << assignment.device);

                connect(child, static_cast<void (QProcess::*)(int)>(&QProcess::finished), this, [this, assignment](int exitCode)
                {
                    LOG_W(LogTag, "Child " + assignment.device + " has exited with code " + QString::number(exitCode));

                    if (_childInterfaces.contains(assignment.device))
                    {
                        delete _childInterfaces[assignment.device];
                        _childInterfaces.remove(assignment.device);
                    }
                    if (_children.contains(assignment.device))
                    {
                        delete _children[assignment.device];
                        _children.remove(assignment.device);
                    }

                    if (_waitingAssignments.contains(assignment.device))
                    {
                        _waitingAssignments.remove(assignment.device);

                        NotificationMessage notifyMsg;
                        notifyMsg.level = NotificationMessage::Level_Error;
                        notifyMsg.title = "Cannot stream " + assignment.cameraName;
                        notifyMsg.message = "Unexpected error - child process died before accepting its stream assignment.";
                        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));

                        reportVideoState();
                    }
                    if (_currentAssignments.contains(assignment.device))
                    {
                        _currentAssignments.remove(assignment.device);

                        NotificationMessage notifyMsg;
                        notifyMsg.level = NotificationMessage::Level_Error;
                        notifyMsg.title = "Error streaming " + assignment.cameraName;
                        notifyMsg.message = "Unexpected error while streaming this device. Try agian.";
                        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));

                        reportVideoState();
                    }
                });
            }
            else
            {
                _waitingAssignments.remove(assignment.device);
                _currentAssignments.remove(assignment.device);

                if (_childInterfaces.contains(assignment.device))
                {
                    // Child for this device is running and can accept assignments
                    giveChildAssignment(assignment);
                    _currentAssignments.insert(assignment.device, assignment);
                    reportVideoState();
                }
                else
                {
                    // There exits a process for this device, however it is still starting up
                    // and may have a previous assignment queued for it. Queue this assignment
                    _waitingAssignments.insert(assignment.device, assignment);
                }
            }
        }
        else
        {
            LOG_I(LogTag, "Received video request, but it's for a different server");
        }
    }
    else if (msg.topic() == "video_state")
    {
        VideoStateMessage videoStateMsg(msg.payload());
        _lastVideoStateMsg = videoStateMsg;
    }
}

VideoServer::Assignment::Assignment()
{
    device = "";
    cameraName = "";
    address = QHostAddress::Null;
    port = 0;
    profile = GStreamerUtil::VideoProfile();
}

VideoServer::~VideoServer()
{
    // Tell all children to stop themselves
    for (QString childName : _children.keys())
    {
        if (_children[childName]->state() == QProcess::Running)
        {
            terminateChild(childName);
        }
    }
}

void VideoServer::terminateChild(QString childName)
{
    if (_children.contains(childName))
    {
        _children[childName]->terminate();
        if (!_children[childName]->waitForFinished(1000))
        {
            _children[childName]->kill();
        }
    }
}

QString VideoServer::findUsbCamera(QString serial, QString productId, QString vendorId, int offset)
{
    // Search for all /dev/video* devices

    QDir dev("/dev");
    QStringList allFiles = dev.entryList(QDir::NoDotAndDotDot | QDir::System | QDir::Files);

    for (QString file : allFiles)
    {
        if (file.contains("video"))
        {
            // Found a new dev device
            QString device = "/dev/" + file;

            // Use udevadm to get info about each /dev/video* device
            QProcess udevadm;
            udevadm.start("udevadm info -a -n " + device);
            udevadm.waitForFinished();
            QString udevadmOutput = QString(udevadm.readAllStandardOutput());

            QLatin1String vendorToken("{idVendor}==\"");
            int vendorIndex = udevadmOutput.indexOf(vendorToken);
            if (vendorIndex >= 0)
            {
                vendorIndex += vendorToken.size();
                QString candidateVendor = udevadmOutput.mid(vendorIndex, udevadmOutput.indexOf("\"", vendorIndex) - vendorIndex);
                if (candidateVendor.trimmed() != vendorId.trimmed()) continue;
            }

            QLatin1String productToken("{idProduct}==\"");
            int productIndex = udevadmOutput.indexOf(productToken);
            if (productIndex >= 0)
            {
                productIndex += productToken.size();
                QString candidateProductId = udevadmOutput.mid(productIndex, udevadmOutput.indexOf("\"", productIndex) - productIndex);
                if (candidateProductId.trimmed() != productId.trimmed()) continue;
            }

            QLatin1String serialToken("{serial}==\"");
            int serialIndex = udevadmOutput.indexOf(serialToken);
            if (serialIndex >= 0)
            {
                serialIndex += serialToken.size();
                QString candidateSerial = udevadmOutput.mid(serialIndex, udevadmOutput.indexOf("\"", serialIndex) - serialIndex);
                if (candidateSerial.trimmed() != serial.trimmed()) continue;
            }

            // Found a matching camera

            if (offset > 0)
            {
                offset--;
                continue;
            }

            return file;
        }
    }
    return "";
}

void VideoServer::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _heartbeatTimerId)
    {
        for (QDBusInterface* interface : _childInterfaces)
        {
            interface->call(QDBus::NoBlock, "heartbeat");
        }
    }
}

void VideoServer::giveChildAssignment(Assignment assignment)
{
    if (_childInterfaces.contains(assignment.device))
    {
        if (assignment.profile.codec == GStreamerUtil::CODEC_NULL)
        {
            _childInterfaces[assignment.device]->call(
                        QDBus::NoBlock,
                        "stop");

        }
        else if (assignment.isStereo)
        {
            _childInterfaces[assignment.device]->call(
                        QDBus::NoBlock,
                        "streamStereo",
                        assignment.device,
                        assignment.device2,
                        assignment.address.toString(),
                        assignment.port,
                        assignment.profile.toString(),
                        assignment.vaapi);
        }
        else
        {
            _childInterfaces[assignment.device]->call(
                        QDBus::NoBlock,
                        "stream",
                        assignment.device,
                        assignment.address.toString(),
                        assignment.port,
                        assignment.profile.toString(),
                        assignment.vaapi);
        }
    }
}

void VideoServer::onChildLogInfo(QString childName, const QString &tag, const QString &message)
{
    LOG_I(QString("[child %1] %2").arg(childName, tag), message);
}

void VideoServer::onChildReady(QString childName)
{
    LOG_I(LogTag, "Child " + childName + " is ready to accept an assignment");

    if (!_childInterfaces.contains(childName))
    {
        // Open a D-Bus interface to this child
        LOG_I(LogTag, "Opening D-Bus interface to child " + childName);
        _childInterfaces.insert(childName, new QDBusInterface(
                                    SORO_DBUS_VIDEO_CHILD_SERVICE_NAME(childName),
                                    "/",
                                    "",
                                    QDBusConnection::sessionBus(),
                                    this));
    }

    if (!_childInterfaces[childName]->isValid())
    {
        LOG_E(LogTag, "Cannot create D-Bus connection to child " + childName + " even though it has a D-Bus conneciton to us");
        terminateChild(childName);

        if (_waitingAssignments.contains(childName))
        {
            NotificationMessage notifyMsg;
            notifyMsg.level = NotificationMessage::Level_Error;
            notifyMsg.title = "Cannot stream " + _waitingAssignments.value(childName).cameraName;
            notifyMsg.message = "Unexpected error - cannot create D-Bus connection to child stream process";
            _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));
            _waitingAssignments.remove(childName);
        }

        reportVideoState();
        return;
    }

    _currentAssignments.remove(childName);

    // Check if there is a stream assignment waiting for this child
    if (_waitingAssignments.contains(childName))
    {
        giveChildAssignment(_waitingAssignments.value(childName));
        _currentAssignments.insert(childName, _waitingAssignments.value(childName));
        _waitingAssignments.remove(childName);
    }

    reportVideoState();
}

void VideoServer::reportVideoState()
{
    VideoStateMessage msg;
    LOG_I(LogTag, "Reporting video state through MQTT...");

    // Add all video state messages from other computers
    for (VideoMessage videoMsg : _lastVideoStateMsg.videoStates)
    {
        if (videoMsg.camera_computerIndex != _settings->getComputerIndex())
        {
            msg.videoStates.append(videoMsg);
        }
    }

    // Add all video state messages from this computer
    for (Assignment videoAssignment : _currentAssignments.values())
    {
        if (videoAssignment.profile.codec != GStreamerUtil::CODEC_NULL)
        {
            msg.videoStates.append(videoAssignment.originalMessage);
            LOG_I(LogTag, " - Appending state for " + videoAssignment.cameraName);
        }
    }

    _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "video_state", msg, 0, true)); // <-- Retain message
}

void VideoServer::onChildError(QString childName, QString message)
{
    LOG_E(LogTag, "Child " + childName + " reports an error: " + message);
    if (_currentAssignments.contains(childName))
    {
        // Send a message on the notification topic
        NotificationMessage notifyMsg;
        notifyMsg.level = NotificationMessage::Level_Error;
        notifyMsg.title = "Error streaming " + _currentAssignments.value(childName).cameraName;
        notifyMsg.message = "Stream error: " + message;

        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "notification", notifyMsg, 0));
        _currentAssignments.remove(childName);
    }
}

void VideoServer::onChildStreaming(QString childName)
{
    LOG_I(LogTag, "Child " + childName + " has started streaming");
}

} // namespace Soro
