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
            ("video_state", 1, &VideoServer::onVideoStateMessage, this);
    if (!_videoStateSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for video_state topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for video_state topic...");
    _videoStatePublisher = _nh.advertise<ros_generated::video_state>("video_state", 1);
    if (!_videoStatePublisher) MainController::panic(LogTag, "Failed to create ROS publisher for video_state topic");

    Logger::logInfo(LogTag, "Creating ROS subscriber for video_request topic...");
    _videoRequestSubscriber = _nh.subscribe
            <ros_generated::video, Soro::VideoServer>
            ("video_request", 1, &VideoServer::onVideoRequestMessage, this);
    if (!_videoStateSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for video_request topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for notification topic...");
    _notificationPublisher = _nh.advertise<ros_generated::notification>("notification", 1);
    if (!_videoStatePublisher) MainController::panic(LogTag, "Failed to create ROS publisher for notification topic");

    // Default to use software encoding for all formats
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_H264, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_H265, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_VP8, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_VP9, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_MPEG2, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_MJPEG, false);
    _useVaapi.insert(GStreamerUtil::VIDEO_CODEC_MPEG4, false);
}

VideoServer::~VideoServer()
{
    // Tell all children to stop themselves
    for (QProcess *process : _children)
    {
        if (process->state() == QProcess::Running)
        {
            stopChild(process->processId());
        }
    }
}

void VideoServer::setShouldUseVaapiForCodec(quint8 codec, bool vaapi)
{
    _useVaapi.insert(codec, vaapi);
}

void VideoServer::spawnChildForAssignment(Assignment assignment)
{
    Logger::logInfo(LogTag, "Spawning new child...");
    QProcess *child = new QProcess(this);
    qint64 *pid = new qint64;
    _children.append(child);

    child->start(SORO_ROVER_VIDEO_STREAM_PROCESS_PATH);

    connect(child, &QProcess::stateChanged, this, [this, child, pid, assignment](QProcess::ProcessState state)
    {
        switch (state)
        {
        case QProcess::Starting:
            break;
        case QProcess::Running:
            *pid = child->processId();
            Logger::logInfo(LogTag, "Child " + QString::number(*pid) + " has started");
            _waitingVideoStreams.insert((qint64)(*pid), assignment);
            break;
        case QProcess::NotRunning:
            // Remove this child from our list of children
            Logger::logInfo(LogTag, "Child " + QString::number(*pid) + " has exited with code " + QString::number(child->exitCode()));
            _children.removeAll(child);
            if (_childInterfaces.contains((qint64)(*pid)))
            {
                delete _childInterfaces[(qint64)(*pid)];
                _childInterfaces.remove((qint64)(*pid));
            }
            if (_waitingVideoStreams.contains((qint64)(*pid)))
            {
                Logger::logError(LogTag, "Child process died before it could be given a stream assignment");
                _waitingVideoStreams.remove((qint64)(*pid));
                ros_generated::notification notifyMsg;
                notifyMsg.type = NOTIFICATION_TYPE_ERROR;
                notifyMsg.title = QString("Cannot stream " + assignment.cameraName).toStdString();
                notifyMsg.message = "Unexpected error - child process died before accepting its stream assignment";
                _notificationPublisher.publish(notifyMsg);
            }
            delete child;
            _assignedVideoStreams.remove((qint64)(*pid));
            delete pid;
            reportVideoState();
            break;
        }
    });
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

void VideoServer::onVideoStateMessage(ros_generated::video_state msg)
{
    _lastVideoStateMsg = msg;
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
            notifyMsg.message = "The definition for this camera does not match any cameras connected to this computer.";
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

        spawnChildForAssignment(assignment);
    }
    else
    {
        Logger::logInfo(LogTag, "Received video request, but it's for a different server");
    }
}

void VideoServer::stopChild(qint64 pid)
{
    if (_childInterfaces.contains(pid))
    {
        _childInterfaces[pid]->call(QDBus::NoBlock, "stop");
    }
}

void VideoServer::killChild(qint64 pid)
{
    for (QProcess *child : _children)
    {
        if (child->processId() == pid)
        {
            child->kill();
        }
    }
}

void VideoServer::onChildReady(qint64 childPid)
{
    Logger::logInfo(LogTag, "onChildReady(): " + QString::number(childPid));
    // Open a D-Bus interface to this child's pid
    if (!_childInterfaces.contains(childPid))
    {
        _childInterfaces.insert(childPid, new QDBusInterface(
                                    SORO_DBUS_VIDEO_CHILD_SERVICE_NAME(QString::number(childPid)),
                                    "/",
                                    "",
                                    QDBusConnection::sessionBus()));
    }

    if (!_childInterfaces[childPid]->isValid())
    {
        Logger::logError(LogTag, "Cannot create D-Bus connection to child " + QString::number(childPid) + " even though it has a D-Bus conneciton to us");
        killChild(childPid);
        _waitingVideoStreams.remove(childPid);

        ros_generated::notification notifyMsg;
        notifyMsg.type = NOTIFICATION_TYPE_ERROR;
        notifyMsg.title = "Unexpected streaming error";
        notifyMsg.message = "Unexpected error - cannot create D-Bus connection to child stream process";
        _notificationPublisher.publish(notifyMsg);

        reportVideoState();
    }

    // Check if there is a stream assignment waiting for this child
    if (_waitingVideoStreams.contains(childPid))
    {
        Logger::logInfo(LogTag, "Calling stream() on child " + QString::number(childPid));
        Assignment assignment = _waitingVideoStreams.value(childPid);
        QDBusReply<void> reply = _childInterfaces[childPid]->call(
                    QDBus::Block,
                    "stream",
                    assignment.device,
                    assignment.address.toString(),
                    assignment.port,
                    assignment.profile.toString(),
                    assignment.vaapi);

        if (!reply.isValid())
        {
            qDebug() << _childInterfaces[childPid]->lastError();
        }

        // This stream is now assigned, update video state
        _waitingVideoStreams.remove(childPid);
        _assignedVideoStreams.insert(childPid, assignment);
        reportVideoState();
    }
    else
    {
        Logger::logWarn(LogTag, "Child process reports ready, however there is no stream waiting for it.");
        stopChild(childPid);
    }
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
    for (Assignment videoAssignment : _assignedVideoStreams.values())
    {
        msg.videoStates.push_back(videoAssignment.originalMessage);
    }

    _videoStatePublisher.publish(msg);
}

void VideoServer::onChildError(qint64 childPid, QString message)
{
    Logger::logError(LogTag, "Child " + QString::number(childPid) + " reports an error: " + message);
    if (_assignedVideoStreams.contains(childPid))
    {
        // Send a message on the notification topic
        ros_generated::notification msg;
        msg.type = NOTIFICATION_TYPE_ERROR;
        msg.title = "Error streaming video";
        msg.message = QString("Error while streaming " + QString(_assignedVideoStreams[childPid].cameraName) + ": " + message).toStdString();

        _notificationPublisher.publish(msg);
    }
}

void VideoServer::onChildStreaming(qint64 childPid)
{
    Logger::logInfo(LogTag, "Child " + QString::number(childPid) + " has started streaming");
}

} // namespace Soro
