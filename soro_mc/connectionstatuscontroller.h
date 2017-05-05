#ifndef CONNECTIONSTATUSCONTROLLER_H
#define CONNECTIONSTATUSCONTROLLER_H

#include <QObject>
#include <QMap>
#include <QTimerEvent>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt32.h>

#include "ros_generated/data_rate.h"

#include <ros/ros.h>

namespace Soro {

/* Handles connection state monitoring for the soro_mc application.
 *
 * This class tracks the following things:
 *  - If we are connected to the rover
 *  - The round-trip latency to the rover
 *  - The bitrate up and down to the rover
 *
 * All of this information is told to us by the master mission control
 * process, which has its own implementation of this class.
 *
 * In addition, since ROS has no utility to measure bitrate, all
 * bits up/down to the rover should be logged using the logBitsUp() and
 * logBitsDown() functions. These rates are then sent to the master from all
 * mission controls, where it is aggregated (and combined with video bitrate)
 * to form the bitrate calculation which can be accessed from this class.
 */
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
    void dataRateUpdate(quint64 rateUp, quint64 rateDown);

public Q_SLOTS:
    void logDataUp(quint32 bytes);
    void logDataDown(quint32 bytes);

    void onNewLatencyMessage(std_msgs::UInt32 msg);
    void onNewDataRateMessage(ros_generated::data_rate msg);
    void setConnected(bool connected);

protected:
    void timerEvent(QTimerEvent *e);

private:
    ros::NodeHandle _nh;
    ros::Subscriber _latencySubscriber;
    ros::Subscriber _dataRateSubscriber;
    ros::Publisher _dataUpPublisher;
    ros::Publisher _dataDownPublisher;

    int _disconnectTimeThreshold;
    int _disconnectWatchdogTimerId;
    bool _connected;
};

} // namespace Soro

#endif // CONNECTIONSTATUSCONTROLLER_H
