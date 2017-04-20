#ifndef PINGWORKER_H
#define PINGWORKER_H

#include <QObject>
#include <QTimer>
#include <ros/ros.h>

namespace Soro {

/* Since ROS services block, this class is used to run on a background thread to regularly
 * query the ping topic and determine latency. It will emit the 'ack' signal when a new
 * latency is calculated, which can be connected to from the UI thread by specifying a
 * connection type of QueuedConnection.
 *
 * This was designed to be used by MasterConnectionStatusController.
 */
class PingWorker: public QObject
{
    Q_OBJECT
public:
    PingWorker(uint interval, QObject *parent=0);

Q_SIGNALS:
    void ack(quint32 millis);

private Q_SLOTS:
    void work();

private:
    ros::NodeHandle _nh;
    ros::ServiceClient _pingClient;
    uint _interval;
    quint64 _nextPing;
};

} // namespace Soro


#endif // PINGWORKER_H
