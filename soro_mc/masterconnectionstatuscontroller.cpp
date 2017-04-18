#include "masterconnectionstatuscontroller.h"
#include "libsoromc/logger.h"

#define LogTag "MasterConnectionStatusController"

namespace Soro {

MasterConnectionStatusController::MasterConnectionStatusController(QObject *parent) : QObject(parent)
{
    Logger::logInfo(LogTag, "Creating ROS subscriber for latency topic...");
    _bitsUpSubscriber = MainController::getNodeHandle()->subscribe
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

void MasterConnectionStatusController::setDisconnectTimeThreshold(int threshold)
{
    _disconnectTimeThreshold = threshold;
}

int MasterConnectionStatusController::getDisconnectTimeThreshold() const
{
    return _disconnectTimeThreshold;
}

bool MasterConnectionStatusController::isConnected() const
{
    return _connected;
}

void MasterConnectionStatusController::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _disconnectWatchdogTimerId)
    {
        setConnected(false);
        killTimer(_disconnectWatchdogTimerId);
        _disconnectWatchdogTimerId = -1;
    }
}

void MasterConnectionStatusController::setConnected(bool connected)
{
    if (connected != _connected) {
        _connected = connected;
        Q_EMIT connectedChanged(_connected);
    }
}

} // namespace Soro
