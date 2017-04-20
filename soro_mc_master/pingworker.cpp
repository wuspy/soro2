#include "pingworker.h"
#include "libsoromc/logger.h"
#include "maincontroller.h"

#include <QDateTime>

#define LogTag "PingWorker"

namespace Soro {

PingWorker::PingWorker(uint interval, QObject *parent): QObject(parent)
{
    Logger::logInfo(LogTag, "Creating ROS client for ping service...");
    _pingClient = _nh.serviceClient<ros_generated::ping>("ping");
    if (!_pingClient) MainController::panic(LogTag, "Failed to create ROS client for ping service");
    _interval = interval;
    _nextPing = 1;
    QTimer::singleShot(_interval, this, &PingWorker::work);
}


// REMINDER: This code is designed to execuce in a background thread!!!
void PingWorker::work()
{
    qint64 start = QDateTime::currentDateTime().toMSecsSinceEpoch();
    ros_generated::ping srv;
    srv.request.ping = _nextPing++;

    uint time = QDateTime::currentDateTime().toMSecsSinceEpoch() - start;
    if (_pingClient.call(srv) && (srv.response.ack == srv.request.ping))
    {
        Q_EMIT ack(time);
    }
    else
    {
        Logger::logWarn(LogTag, "Could not call ping service");
    }
    QTimer::singleShot(qMax<int>(_interval - time, 10), this, &PingWorker::work);
}

} // namespace Soro
