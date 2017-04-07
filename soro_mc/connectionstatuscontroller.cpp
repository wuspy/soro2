#include "connectionstatuscontroller.h"
#include "maincontroller.h"
#include "libsoromc/logger.h"

#define LogTag "ConnectionStatusController"

namespace Soro {

ConnectionStatusController::ConnectionStatusController(QObject *parent) : QObject(parent)
{
    Logger::logInfo(LogTag, "Creating ROS publisher and subscriber...");
    _pingPublisher = MainController::getNodeHandle()->advertise<Soro::Messages::ping>("ping", 1);
    _ackSubscriber = MainController::getNodeHandle()->subscribe
            <Soro::Messages::ping, Soro::ConnectionStatusController>
            ("ack", 1, &ConnectionStatusController::onNewAckMessage, this);
    Logger::logInfo(LogTag, "ROS publisher and subscriber created");

    _bitsDown = _bitsUp = 0;
    _updateBitrateTimerId = -1;
    _sendPingTimerId = -1;
    _currentPingId = 0;
    _disconnectWatchdogTimerId = -1;
    setDisconnectTimeThreshold(2000);
    setBitrateUpdateInterval(500);
    setPingRequestInterval(500);
}

void ConnectionStatusController::setBitrateUpdateInterval(int interval)
{
    if (_updateBitrateTimerId != -1) killTimer(_updateBitrateTimerId);
    _bitrateUpdateInterval = interval;
    _updateBitrateTimerId = startTimer(_bitrateUpdateInterval);
    Logger::logInfo(LogTag, "Bitrate update interval is now set at " + QString::number(interval));
}

int ConnectionStatusController::getBitrateUpdateInterval() const
{
    return _bitrateUpdateInterval;
}

void ConnectionStatusController::setPingRequestInterval(int interval)
{
    if (_sendPingTimerId != -1) killTimer(_sendPingTimerId);
    _pingRequestInterval = interval;
    _sendPingTimerId = startTimer(_pingRequestInterval);
    Logger::logInfo(LogTag, "Ping request interval is now set at " + QString::number(interval));
}

int ConnectionStatusController::getPingRequestInterval() const
{
    return _pingRequestInterval;
}

void ConnectionStatusController::setDisconnectTimeThreshold(int threshold)
{
    _disconnectTimeThreshold = threshold;
}

int ConnectionStatusController::getDisconnectTimeThreshold() const
{
    return _disconnectTimeThreshold;
}

void ConnectionStatusController::onNewAckMessage(Soro::Messages::ping msg)
{
    logBitsDown(strlen((msg.sender.c_str()) + 8) * 8);
    if (QString(msg.sender.c_str()) != MainController::getMissionControlId()) return; // Not our message

    // Stop the disconnect watchdog
    if (_disconnectWatchdogTimerId != -1)
    {
        killTimer(_disconnectWatchdogTimerId);
        _disconnectWatchdogTimerId = -1;
    }
    setConnected(true);

    int latency = _pingTimes.value((quint64)msg.id, -1);
    if (latency == -1) return; // The message ID wasn't in our time table for some reason
    _pingTimes.remove((quint64)msg.id);
    latency = QDateTime::currentDateTime().toMSecsSinceEpoch() - latency;
    Logger::logInfo(LogTag, "Latency: " + QString::number(latency));

    emit latencyUpdate(latency);
}

void ConnectionStatusController::logBitsDown(int bits)
{
    _bitsDown += bits;
}

void ConnectionStatusController::logBitsUp(int bits)
{
    _bitsUp += bits;
}

bool ConnectionStatusController::isConnected() const
{
    return _connected;
}

void ConnectionStatusController::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _updateBitrateTimerId)
    {
        int bitsUpRate = _bitsUp / _bitrateUpdateInterval;
        int bitsDownRate = _bitsDown / _bitrateUpdateInterval;
        _bitsUp = 0;
        _bitsDown = 0;

        emit bitrateUpdate(bitsUpRate, bitsDownRate);
    }
    else if (e->timerId() == _sendPingTimerId)
    {
        Soro::Messages::ping msg;
        msg.sender = MainController::getMissionControlId().toStdString();
        msg.id = _currentPingId++;
        _pingPublisher.publish(msg);
        logBitsUp(strlen((msg.sender.c_str()) + 8) * 8);
        if (_pingTimes.size() > 100)
        {
            // Prune old values from time table
            QList<quint64> keys = _pingTimes.keys();
            for (int i = 0; i < 50; ++i)
            {
                _pingTimes.remove(keys[i]);
            }
        }
        // Start the disconnect watchdog if it isn't already started
        if (_connected && (_disconnectWatchdogTimerId == -1))
        {
            _disconnectWatchdogTimerId = startTimer(_disconnectTimeThreshold);
        }
        _pingTimes.insert(msg.id, QDateTime::currentDateTime().toMSecsSinceEpoch());
    }
    else if (e->timerId() == _disconnectWatchdogTimerId)
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
        emit connectedChanged(_connected);
    }
}

} // namespace Soro
