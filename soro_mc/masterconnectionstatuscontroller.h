#ifndef MASTERCONNECTIONSTATUSCONTROLLER_H
#define MASTERCONNECTIONSTATUSCONTROLLER_H

#include <QObject>

#include <ros/ros.h>

namespace Soro {

class MasterConnectionStatusController : public QObject
{
    Q_OBJECT
public:
    explicit MasterConnectionStatusController(QObject *parent = 0);

    bool isConnected() const;

Q_SIGNALS:
    void latencyUpdated(quint32 latency);
    void bitrateUpdated(quint64 bitrateUp, quint64 bitrateDown);
    void connectedChanged(bool connected);

private:
    void setConnected(bool connected);

    ros::Publisher _bitratePublisher;
    ros::Publisher _latencyPublisher;
    ros::Subscriber _bitsDownSubscriber;
    ros::Subscriber _bitsUpSubscriber;
    ros::ServiceClient _pingClient;

    int _disconnectTimeThreshold;
    int _disconnectWatchdogTimerId;
    bool _connected;
};

} // namespace Soro

#endif // MASTERCONNECTIONSTATUSCONTROLLER_H
