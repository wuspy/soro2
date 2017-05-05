#include "videoserver.h"
#include "soro_core/constants.h"
#include "soro_core/logger.h"
#include "soro_core/gstreamerutil.h"

#include "maincontroller.h"

#define LogTag "VideoServer"

namespace Soro {

VideoServer::VideoServer(int computerIndex, const RosNodeList *rosNodeList, QObject *parent) : QObject(parent)
{
    _computerIndex = computerIndex;
    _rosNodeList = rosNodeList;

    // Ensure child process executable exits
    if (!QFile(SORO_ROVER_VIDEO_STREAM_PROCESS_PATH).exists())
    {
        MainController::panic(LogTag, "Video stream process is not at the correct path");
    }

    Logger::logInfo(LogTag, "Registering as D-Bus RPC object...");
    // Register this class as a D-Bus RPC service so other processes can call our public slots
    QDBusConnection::sessionBus().registerObject("/", this, QDBusConnection::ExportAllSlots);

    Logger::logInfo(LogTag, "Creating ROS subscriber for video_state topic...");
    _videoStateSubscriber = _nh.subscribe
            <ros_generated::video_state, Soro::VideoServer>
            ("video_state", 100, &VideoServer::onVideoStateMessage, this);
    if (!_videoStateSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for video_state topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for video_state topic...");
    _videoStatePublisher = _nh.advertise<ros_generated::video_state>("video_state", 100, true); // <<--- LATCH
    if (!_videoStatePublisher) MainController::panic(LogTag, "Failed to create ROS publisher for video_state topic");

    Logger::logInfo(LogTag, "Creating ROS subscriber for video_request topic...");
    _videoRequestSubscriber = _nh.subscribe
            <ros_generated::video, Soro::VideoServer>
            ("video_request", 100, &VideoServer::onVideoRequestMessage, this);
    if (!_videoRequestSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for video_request topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for notification topic...");
    _notificationPublisher = _nh.advertise<ros_generated::notification>("notification", 1);
    if (!_notificationPublisher) MainController::panic(LogTag, "Failed to create ROS publisher for notification topic");

    // Default to use software encoding for all formats
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_H264, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_H265, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_VP8, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_VP9, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_MPEG2, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_MJPEG, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_MPEG4, false);

    // Start heartbeat timer so children still know we're running
    _heartbeatTimerId = startTimer(1000);
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

void VideoServer::setShouldUseVaapiForCodec(quint8 codec, bool vaapi)
{
    _useVaapi.insert(codec, vaapi);
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

void VideoServer::onVideoStateMessage(ros_generated::video_state msg)
{
    _lastVideoStateMsg = msg;
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

void VideoServer::onVideoRequestMessage(ros_generated::video msg)
{
    if (msg.camera_computerIndex == _computerIndex)
    {
        Logger::logInfo(LogTag, "Received new video request for this server");
        QHostAddress mcAddress = _rosNodeList->getAddressForNode("/mc_master");
        if (mcAddress == QHostAddress::Null)
        {
            // Master mission control isn't running, we can't stream video anywhere
            Logger::logError(LogTag, "Master mission control is not connected, unable to stream video");
            ros_generated::notification notifyMsg;
            notifyMsg.type = NOTIFICATION_TYPE_ERROR;
            notifyMsg.title = QString("Cannot stream " + QString(msg.camera_name.c_str())).toStdString();
            notifyMsg.message = "The master mission control program was not found on the network, so video can't be streamed to mission control.";
            _notificationPublisher.publish(notifyMsg);
            return;
        }

        QString cameraDevice = findUsbCamera(QString(msg.camera_matchSerial.c_str()),
                                             QString(msg.camera_matchProductId.c_str()),
                                             QString(msg.camera_matchVendorId.c_str()),
                                             msg.camera_offset);
        if (cameraDevice.isEmpty())
        {
            // This camera wasn't found on our computer. Notify mission control
            Logger::logError(LogTag, "Requested camera definition does not exist on this computer");
            ros_generated::notification notifyMsg;
            notifyMsg.type = NOTIFICATION_TYPE_ERROR;
            notifyMsg.title = QString("Cannot stream " + QString(msg.camera_name.c_str())).toStdString();
            notifyMsg.message = "The definition for this camera does not match any cameras connected to this server.";
            _notificationPublisher.publish(notifyMsg);
            return;
        }

        Assignment assignment;
        assignment.device = cameraDevice;
        assignment.cameraName = QString(msg.camera_name.c_str());
        assignment.vaapi = _useVaapi.value(msg.encoding);
        assignment.profile = GStreamerUtil::VideoProfile(msg);
        assignment.address = mcAddress;
        assignment.port = SORO_NET_FIRST_VIDEO_PORT + msg.camera_index;
        assignment.originalMessage = msg;

        if (!_children.contains(assignment.device))
        {
            // Spawn a new child, and queue this assignment to be executed when the child is ready
            Logger::logInfo(LogTag, "Spawning new child for " + assignment.device + "...");
            QProcess *child = new QProcess(this);
            _children.insert(assignment.device, child);
            _waitingAssignments.insert(assignment.device, assignment);

            // The first argument to the process, representing the child's name, is the video device
            // it is assigned to work on
            child->start(SORO_ROVER_VIDEO_STREAM_PROCESS_PATH, QStringList() << assignment.device);

            connect(child, static_cast<void (QProcess::*)(int)>(&QProcess::finished), this, [this, assignment](int exitCode)
            {
                Logger::logWarn(LogTag, "Child " + assignment.device + " has exited with code " + QString::number(exitCode));

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

                    ros_generated::notification notifyMsg;
                    notifyMsg.type = NOTIFICATION_TYPE_ERROR;
                    notifyMsg.title = QString("Cannot stream " + assignment.cameraName).toStdString();
                    notifyMsg.message = "Unexpected error - child process died before accepting its stream assignment.";
                    _notificationPublisher.publish(notifyMsg);

                    reportVideoState();
                }
                if (_currentAssignments.contains(assignment.device))
                {
                    _currentAssignments.remove(assignment.device);

                    ros_generated::notification notifyMsg;
                    notifyMsg.type = NOTIFICATION_TYPE_ERROR;
                    notifyMsg.title = QString("Error streaming " + assignment.cameraName).toStdString();
                    notifyMsg.message = "Unexpected error while streaming this device. Try agian.";
                    _notificationPublisher.publish(notifyMsg);

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
        Logger::logInfo(LogTag, "Received video request, but it's for a different server");
    }
}

void VideoServer::onChildLogInfo(QString childName, const QString &tag, const QString &message)
{
    Logger::logInfo(QString("[child %1] %2").arg(childName, tag), message);
}

void VideoServer::onChildReady(QString childName)
{
    Logger::logInfo(LogTag, "Child " + childName + " is ready to accept an assignment");

    if (!_childInterfaces.contains(childName))
    {
        // Open a D-Bus interface to this child
        Logger::logInfo(LogTag, "Opening D-Bus interface to child " + childName);
        _childInterfaces.insert(childName, new QDBusInterface(
                                    SORO_DBUS_VIDEO_CHILD_SERVICE_NAME(childName),
                                    "/",
                                    "",
                                    QDBusConnection::sessionBus(),
                                    this));
    }

    if (!_childInterfaces[childName]->isValid())
    {
        Logger::logError(LogTag, "Cannot create D-Bus connection to child " + childName + " even though it has a D-Bus conneciton to us");
        terminateChild(childName);

        if (_waitingAssignments.contains(childName))
        {
            ros_generated::notification notifyMsg;
            notifyMsg.type = NOTIFICATION_TYPE_ERROR;
            notifyMsg.title = QString("Cannot stream " + _waitingAssignments.value(childName).cameraName).toStdString();
            notifyMsg.message = "Unexpected error - cannot create D-Bus connection to child stream process";
            _notificationPublisher.publish(notifyMsg);
            _waitingAssignments.remove(childName);
        }

        reportVideoState();
        return;
    }

    // Check if there is a stream assignment waiting for this child
    if (_waitingAssignments.contains(childName))
    {
        giveChildAssignment(_waitingAssignments.value(childName));
        _currentAssignments.insert(childName, _waitingAssignments.value(childName));
        _waitingAssignments.remove(childName);
    }
    if (_currentAssignments.contains(childName))
    {
        // This child stopped while streaming a camera
        _currentAssignments.remove(childName);
    }

    reportVideoState();
}

void VideoServer::reportVideoState()
{
    ros_generated::video_state msg;

    // Add all video state messages from other computers
    for (ros_generated::video videoMsg : _lastVideoStateMsg.videoStates)
    {
        if (videoMsg.camera_computerIndex != _computerIndex)
        {
            msg.videoStates.push_back(videoMsg);
        }
    }

    // Add all video state messages from this computer
    for (Assignment videoAssignment : _currentAssignments.values())
    {
        if (videoAssignment.profile.codec != GStreamerUtil::CODEC_NULL)
        {
            msg.videoStates.push_back(videoAssignment.originalMessage);
        }
    }

    _videoStatePublisher.publish(msg);
}

void VideoServer::onChildError(QString childName, QString message)
{
    Logger::logError(LogTag, "Child " + childName + " reports an error: " + message);
    if (_currentAssignments.contains(childName))
    {
        // Send a message on the notification topic
        ros_generated::notification msg;
        msg.type = NOTIFICATION_TYPE_ERROR;
        msg.title = QString("Error streaming " + _currentAssignments.value(childName).cameraName).toStdString();
        msg.message = QString("Stream error: " + message).toStdString();

        _notificationPublisher.publish(msg);
        _currentAssignments.remove(childName);
    }
}

void VideoServer::onChildStreaming(QString childName)
{
    Logger::logInfo(LogTag, "Child " + childName + " has started streaming");
}

} // namespace Soro
