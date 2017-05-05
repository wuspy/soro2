#include "audioserver.h"
#include "soro_core/constants.h"
#include "soro_core/logger.h"
#include "soro_core/gstreamerutil.h"

#include "maincontroller.h"

#define LogTag "VideoServer"

namespace Soro {

AudioServer::AudioServer(const RosNodeList *rosNodeList, QObject *parent) : QObject(parent)
{
    _rosNodeList = rosNodeList;

    // Ensure child process executable exits
    if (!QFile(SORO_ROVER_AUDIO_STREAM_PROCESS_PATH).exists())
    {
        MainController::panic(LogTag, "Video stream process is not at the correct path");
    }

    Logger::logInfo(LogTag, "Registering as D-Bus RPC object...");
    // Register this class as a D-Bus RPC service so other processes can call our public slots
    QDBusConnection::sessionBus().registerObject("/", this, QDBusConnection::ExportAllSlots);

    Logger::logInfo(LogTag, "Creating ROS publisher for audio_state topic...");
    _audioStatePublisher = _nh.advertise<ros_generated::audio>("audio_state", 1, true); // <<--- LATCH
    if (!_audioStatePublisher) MainController::panic(LogTag, "Failed to create ROS publisher for audio_state topic");

    Logger::logInfo(LogTag, "Creating ROS subscriber for audio_request topic...");
    _audioRequestSubscriber = _nh.subscribe
            <ros_generated::audio, Soro::AudioServer>
            ("audio_request", 1, &AudioServer::onAudioRequestMessage, this);
    if (!_audioRequestSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for audio_request topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for notification topic...");
    _notificationPublisher = _nh.advertise<ros_generated::notification>("notification", 1);
    if (!_notificationPublisher) MainController::panic(LogTag, "Failed to create ROS publisher for notification topic");

    // Start heartbeat timer so children still know we're running
    _heartbeatTimerId = startTimer(1000);
    _child = nullptr;
    _childInterface = nullptr;
    _hasWaitingAssignment = false;
    _hasCurrentAssignment = false;
}

AudioServer::Assignment::Assignment()
{
    address = QHostAddress::Null;
    port = 0;
    profile = GStreamerUtil::AudioProfile();
}

AudioServer::~AudioServer()
{
    terminateChild();
}

void AudioServer::terminateChild()
{
    if (_child && (_child->state() != QProcess::NotRunning))
    {
        _child->terminate();
        if (!_child->waitForFinished(1000))
        {
            _child->kill();
        }
    }
}

void AudioServer::timerEvent(QTimerEvent *e)
{
    if ((e->timerId() == _heartbeatTimerId) && _childInterface)
    {
        _childInterface->call(QDBus::NoBlock, "heartbeat");
    }
}

void AudioServer::giveChildAssignment(Assignment assignment)
{
    if (_childInterface)
    {
        if (assignment.profile.codec == GStreamerUtil::CODEC_NULL)
        {
            _childInterface->call(
                        QDBus::NoBlock,
                        "stop");

            _hasCurrentAssignment = false;
        }
        else
        {
            _childInterface->call(
                        QDBus::NoBlock,
                        "stream",
                        assignment.address.toString(),
                        assignment.port,
                        assignment.profile.toString());

            _hasCurrentAssignment = true;
            _currentAssignment = assignment;
        }

        reportAudioState();
    }
}

void AudioServer::onAudioRequestMessage(ros_generated::audio msg)
{
    Logger::logInfo(LogTag, "Received new audio request for this server");
    QHostAddress mcAddress = _rosNodeList->getAddressForNode("/mc_master");
    if (mcAddress == QHostAddress::Null)
    {
        // Master mission control isn't running, we can't stream video anywhere
        Logger::logError(LogTag, "Master mission control is not connected, unable to stream video");
        ros_generated::notification notifyMsg;
        notifyMsg.type = NOTIFICATION_TYPE_ERROR;
        notifyMsg.title = "Cannot stream audio";
        notifyMsg.message = "The master mission control program was not found on the network, so audio can't be streamed to mission control.";
        _notificationPublisher.publish(notifyMsg);
        return;
    }

    Assignment assignment;
    assignment.profile = GStreamerUtil::AudioProfile(msg);
    assignment.address = mcAddress;
    assignment.port = SORO_NET_AUDIO_PORT;
    assignment.originalMessage = msg;

    if (!_child)
    {
        // Spawn a new child, and queue this assignment to be executed when the child is ready
        Logger::logInfo(LogTag, "Spawning new child");
        _child = new QProcess(this);

        _hasWaitingAssignment = true;
        _waitingAssignment = assignment;

        _child->start(SORO_ROVER_AUDIO_STREAM_PROCESS_PATH);

        connect(_child, static_cast<void (QProcess::*)(int)>(&QProcess::finished), this, [this, assignment](int exitCode)
        {
            Logger::logWarn(LogTag, "Child has exited with code " + QString::number(exitCode));

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

                ros_generated::notification notifyMsg;
                notifyMsg.type = NOTIFICATION_TYPE_ERROR;
                notifyMsg.title = "Cannot stream audio";
                notifyMsg.message = "Unexpected error - child process died before accepting its stream assignment.";
                _notificationPublisher.publish(notifyMsg);

                reportAudioState();
            }
            if (_hasCurrentAssignment)
            {
                _hasCurrentAssignment = false;

                ros_generated::notification notifyMsg;
                notifyMsg.type = NOTIFICATION_TYPE_ERROR;
                notifyMsg.title = "Error streaming audio";
                notifyMsg.message = "Unexpected error while streaming this device. Try agian.";
                _notificationPublisher.publish(notifyMsg);

                reportAudioState();
            }
        });
    }
    else
    {
        _hasWaitingAssignment = false;
        _hasCurrentAssignment = false;

        if (_childInterface)
        {
            // Child for this device is running and can accept assignments
            giveChildAssignment(assignment);
        }
        else
        {
            // There exits a process for this device, however it is still starting up
            // and may have a previous assignment queued for it. Queue this assignment
            _hasWaitingAssignment = true;
            _waitingAssignment = assignment;
        }
    }
}

void AudioServer::onChildLogInfo(const QString &tag, const QString &message)
{
    Logger::logInfo(QString("[child] %2").arg(tag), message);
}

void AudioServer::onChildReady()
{
    Logger::logInfo(LogTag, "Child is ready to accept an assignment");

    if (!_childInterface)
    {
        // Open a D-Bus interface to this child
        Logger::logInfo(LogTag, "Opening D-Bus interface to child");
        _childInterface = new QDBusInterface(
                                    SORO_DBUS_AUDIO_CHILD_SERVICE_NAME,
                                    "/",
                                    "",
                                    QDBusConnection::sessionBus(),
                                    this);
    }

    if (!_childInterface->isValid())
    {
        Logger::logError(LogTag, "Cannot create D-Bus connection to child even though it has a D-Bus conneciton to us");
        terminateChild();

        delete _childInterface;
        _childInterface = nullptr;

        if (_hasWaitingAssignment)
        {
            ros_generated::notification notifyMsg;
            notifyMsg.type = NOTIFICATION_TYPE_ERROR;
            notifyMsg.title = "Cannot stream audio";
            notifyMsg.message = "Unexpected error - cannot create D-Bus connection to child stream process";
            _notificationPublisher.publish(notifyMsg);
        }

        reportAudioState();
        return;
    }

    // Check if there is a stream assignment waiting
    if (_hasWaitingAssignment)
    {
        giveChildAssignment(_waitingAssignment);
    }
    else
    {
        // Ensure there is no current assignment still logged
        _hasCurrentAssignment = false;
    }
}

void AudioServer::reportAudioState()
{
    if (_hasCurrentAssignment)
    {
        _audioStatePublisher.publish(_currentAssignment.originalMessage);
    }
    else
    {
        _audioStatePublisher.publish(GStreamerUtil::AudioProfile().toRosMessage());
    }
}

void AudioServer::onChildError(QString message)
{
    Logger::logError(LogTag, "Child reports an error: " + message);
    if (_hasCurrentAssignment)
    {
        // Send a message on the notification topic
        ros_generated::notification msg;
        msg.type = NOTIFICATION_TYPE_ERROR;
        msg.title = "Error streaming audio";
        msg.message = QString("Stream error: " + message).toStdString();

        _notificationPublisher.publish(msg);
        _hasCurrentAssignment = false;
    }
}

void AudioServer::onChildStreaming()
{
    Logger::logInfo(LogTag, "Child has started streaming");
}

} // namespace Soro
