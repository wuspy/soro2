#ifndef CONNECTIONSTATUSCONTROLLER_H
#define CONNECTIONSTATUSCONTROLLER_H

#include <QObject>
#include <QMap>
#include <QTimerEvent>

#include <libsoromc/pingmessage.h>

#include <ros/ros.h>

namespace Soro {

class ConnectionStatusController : public QObject
{
    Q_OBJECT
public:
    explicit ConnectionStatusController(QObject *parent = 0);

    void setBitrateUpdateInterval(int interval);
    int getBitrateUpdateInterval() const;

    void setPingRequestInterval(int interval);
    int getPingRequestInterval() const;

    void setDisconnectTimeThreshold(int threshold);
    int getDisconnectTimeThreshold() const;

    bool isConnected() const;

public slots:
    void logBitsDown(int bits);
    void logBitsUp(int bits);

signals:
    void latencyUpdate(int latency);
    void connectedChanged(bool connected);
    void bitrateUpdate(int bitrateUp, int bitrateDown);

protected:
    void timerEvent(QTimerEvent *e);

private:
    void onNewAckMessage(Soro::Messages::ping msg);
    void setConnected(bool connected);

    ros::Publisher _pingPublisher;
    ros::Subscriber _ackSubscriber;
    QMap<quint64, qint64> _pingTimes;
    int _bitsDown, _bitsUp;
    int _updateBitrateTimerId;
    int _bitrateUpdateInterval;
    int _sendPingTimerId;
    int _pingRequestInterval;
    int _currentPingId;
    int _disconnectTimeThreshold;
    int _disconnectWatchdogTimerId;
    bool _connected;
};

} // namespace Soro

#endif // CONNECTIONSTATUSCONTROLLER_H
