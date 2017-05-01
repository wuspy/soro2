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
    void bitrateUpdate(quint64 bitrateUp, quint64 bitrateDown);

public Q_SLOTS:
    void logBitsUp(quint32 bits);
    void logBitsDown(quint32 bits);

    void onNewLatencyMessage(std_msgs::UInt32 msg);
    void onNewBitrateMessage(ros_generated::bitrate msg);
    void setConnected(bool connected);

protected:
    void timerEvent(QTimerEvent *e);

private:
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
