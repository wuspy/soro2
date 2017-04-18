#ifndef MASTERCONNECTIONSTATUSCONTROLLER_H
#define MASTERCONNECTIONSTATUSCONTROLLER_H

#include <QObject>
#include <QMap>
#include <QTimerEvent>
#include <QThread>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt32.h>
#include <ros_generated/ping.h>

#include "ros_generated/bitrate.h"
#include "pingworker.h"
#include "settingsmodel.h"

#include <ros/ros.h>

namespace Soro {

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
    void bitrateUpdate(quint64 bitrateUp, quint64 bitrateDown);

public Q_SLOTS:
    void logBitsUp(quint32 bits);
    void logBitsDown(quint32 bits);

private Q_SLOTS:
    void onNewLatency(quint32 latency);

protected:
    void timerEvent(QTimerEvent *e);

private:
    void onNewBitsDownMessage(std_msgs::UInt32 msg);
    void onNewBitsUpMessage(std_msgs::UInt32 msg);
    void setConnected(bool connected);

    ros::NodeHandle _nh;
    ros::ServiceClient _pingClient;
    ros::Publisher _latencyPublisher;
    ros::Publisher _bitratePublisher;
    ros::Subscriber _bitsUpSubscriber;
    ros::Subscriber _bitsDownSubscriber;

    int _disconnectWatchdogTimerId;
    int _bitrateCalcTimerId;
    bool _connected;

    const SettingsModel *_settings;
    quint64 _bitsDown;
    quint64 _bitsUp;
    QThread _workerThread;
    PingWorker *_pingWorker = nullptr;
};

} // namespace Soro

#endif // MASTERCONNECTIONSTATUSCONTROLLER_H
