#include "connectionstatuscontroller.h"
#include "maincontroller.h"
#include "libsoromc/logger.h"

#define LogTag "ConnectionStatusController"

namespace Soro {

ConnectionStatusController::ConnectionStatusController(QObject *parent) : QObject(parent)
{
    Logger::logInfo(LogTag, "Creating ROS subscriber for latency topic...");
    _latencySubscriber = MainController::getNodeHandle()->subscribe
            <std_msgs::UInt32, Soro::ConnectionStatusController>
            ("latency", 1, &ConnectionStatusController::onNewLatencyMessage, this);

    Logger::logInfo(LogTag, "Creating ROS subscriber for bitrate topic...");
    _bitrateSubscriber = MainController::getNodeHandle()->subscribe
            <message_gen::bitrate, Soro::ConnectionStatusController>
            ("bitrate", 1, &ConnectionStatusController::onNewBitrateMessage, this);

    Logger::logInfo(LogTag, "Creating ROS publisher for bits_up_log topic...");
    _bitsUpPublisher = MainController::getNodeHandle()->advertise<std_msgs::UInt32>("bits_up_log", 1);

    Logger::logInfo(LogTag, "Creating ROS publisher for bits_down_log topic...");
    _bitsDownPublisher = MainController::getNodeHandle()->advertise<std_msgs::UInt32>("bits_down_log", 1);

    Logger::logInfo(LogTag, "All ROS publishers and subscribers created");

    _disconnectWatchdogTimerId = -1;
    setDisconnectTimeThreshold(2000);
    _connected = false;
}

void ConnectionStatusController::setDisconnectTimeThreshold(int threshold)
{
    _disconnectTimeThreshold = threshold;
}

int ConnectionStatusController::getDisconnectTimeThreshold() const
{
    return _disconnectTimeThreshold;
}

void ConnectionStatusController::onNewBitrateMessage(message_gen::bitrate msg)
{
    Q_EMIT bitrateUpdate((quint64)msg.bitrateUp, (quint64)msg.bitrateDown);
}

void ConnectionStatusController::onNewLatencyMessage(std_msgs::UInt32 msg)
{
    if (_disconnectWatchdogTimerId != -1) {
        killTimer(_disconnectWatchdogTimerId);
    }
    _disconnectWatchdogTimerId = startTimer(_disconnectTimeThreshold);
    setConnected(true);

    Q_EMIT latencyUpdate((quint32)msg.data);
}

void ConnectionStatusController::logBitsDown(quint32 bits)
{
    std_msgs::UInt32 msg;
    msg.data = bits;
    _bitsDownPublisher.publish<std_msgs::UInt32>(msg);
}

void ConnectionStatusController::logBitsUp(quint32 bits)
{
    std_msgs::UInt32 msg;
    msg.data = bits;
    _bitsUpPublisher.publish<std_msgs::UInt32>(msg);
}

bool ConnectionStatusController::isConnected() const
{
    return _connected;
}

void ConnectionStatusController::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _disconnectWatchdogTimerId)
    {
        setConnected(false);
        killTimer(_disconnectWatchdogTimerId);
        _disconnectWatchdogTimerId = -1;
    }
}

void ConnectionStatusController::setConnected(bool connected)
{
    if (connected != _connected) {
        _connected = connected;
        Q_EMIT connectedChanged(_connected);
    }
}

} // namespace Soro
