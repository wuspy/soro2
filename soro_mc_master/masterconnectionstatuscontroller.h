#ifndef MASTERCONNECTIONSTATUSCONTROLLER_H
#define MASTERCONNECTIONSTATUSCONTROLLER_H

#include <QObject>
#include <QMap>
#include <QTimerEvent>
#include <QThread>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt32.h>
#include <ros_generated/ping.h>

#include "ros_generated/data_rate.h"
#include "pingworker.h"
#include "settingsmodel.h"

#include <ros/ros.h>

namespace Soro {

/* Master implementation of the connection status controller.
 *
 * This class tracks the following things:
 *  - If we are connected to the rover
 *  - The round-trip latency to the rover
 *  - The bitrate up and down to the rover
 *
 * The latency is calculated by calling the ping service through ROS, using
 * a PingWorker class. Connected status is determined by this latency; if we
 * do not get a latency update from PingWorker in the alloted time, we will
 * consider the rover disconnected for the time being. The latency is sent
 * to the latency topic as soon as we get it to forward it to other mission
 * controls.
 *
 * Bitrate is calculated based on two things:
 *  - The bit logs we get from the bits_up_log and bits_down_log topics
 *  - Bits logged using the logBitsUp() and logBitsDown() functions here
 *
 * The bitrate is calculated periodically from these bit logs, and is then emitted
 * from the bitrateUpdate() signal and send to the bitrate ROS topic.
 */
class MasterConnectionStatusController : public QObject
{
    Q_OBJECT
public:
    explicit MasterConnectionStatusController(const SettingsModel *settings, QObject *parent = 0);
    ~MasterConnectionStatusController();

    bool isConnected() const;

Q_SIGNALS:
    void latencyUpdate(quint32 latency);
    void connectedChanged(bool connected);
    void dataRateUpdate(quint64 rateUp, quint64 rateDown);

public Q_SLOTS:
    void logDataUp(quint32 bytes);
    void logDataDown(quint32 bytes);

private Q_SLOTS:
    void onNewLatency(quint32 latency);

protected:
    void timerEvent(QTimerEvent *e);

private:
    void onNewDataDownMessage(std_msgs::UInt32 msg);
    void onNewDataUpMessage(std_msgs::UInt32 msg);
    void setConnected(bool connected);

    ros::NodeHandle _nh;
    ros::ServiceClient _pingClient;
    ros::Publisher _latencyPublisher;
    ros::Publisher _dataRatePublisher;
    ros::Subscriber _dataUpSubscriber;
    ros::Subscriber _dataDownSubscriber;

    int _disconnectWatchdogTimerId;
    int _dataRateCalcTimerId;
    bool _connected;

    const SettingsModel *_settings;
    quint64 _bytesDown;
    quint64 _bytesUp;
    QThread _workerThread;
    PingWorker *_pingWorker = nullptr;
};

} // namespace Soro

#endif // MASTERCONNECTIONSTATUSCONTROLLER_H
