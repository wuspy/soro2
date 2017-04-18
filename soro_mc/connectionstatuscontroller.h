#ifndef CONNECTIONSTATUSCONTROLLER_H
#define CONNECTIONSTATUSCONTROLLER_H

#include <QObject>
#include <QMap>
#include <QTimerEvent>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt32.h>

#include <ros_generated/bitrate.h>

#include <ros/ros.h>

namespace Soro {

class ConnectionStatusController : public QObject
{
    Q_OBJECT
public:
    explicit ConnectionStatusController(QObject *parent = 0);

    void setDisconnectTimeThreshold(int threshold);
    int getDisconnectTimeThreshold() const;

    bool isConnected() const;

Q_SIGNALS:
    void latencyUpdate(quint32 latency);
    void connectedChanged(bool connected);
    void bitrateUpdate(quint64 bitrateUp, quint64 bitrateDown);

public Q_SLOTS:
    void logBitsUp(quint32 bits);
    void logBitsDown(quint32 bits);

protected:
    void timerEvent(QTimerEvent *e);

private:
    void onNewLatencyMessage(std_msgs::UInt32 msg);
    void onNewBitrateMessage(ros_generated::bitrate msg);
    void setConnected(bool connected);

    ros::NodeHandle _nh;
    ros::Subscriber _latencySubscriber;
    ros::Subscriber _bitrateSubscriber;
    ros::Publisher _bitsUpPublisher;
    ros::Publisher _bitsDownPublisher;

    int _disconnectTimeThreshold;
    int _disconnectWatchdogTimerId;
    bool _connected;
};

} // namespace Soro

#endif // CONNECTIONSTATUSCONTROLLER_H
