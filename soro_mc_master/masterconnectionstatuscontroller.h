#ifndef MASTERCONNECTIONSTATUSCONTROLLER_H
#define MASTERCONNECTIONSTATUSCONTROLLER_H

#include <QObject>
#include <QQueue>
#include <QTimer>
#include <QThread>

#include "settingsmodel.h"
#include "qmqtt/qmqtt.h"


namespace Soro {

/* Master implementation of the connection status controller.
 */
class MasterConnectionStatusController : public QObject
{
    Q_OBJECT
public:
    explicit MasterConnectionStatusController(QHostAddress brokerAddress, quint16 brokerPort, int pingInterval, int dataRateCalcInterval, QObject *parent = 0);

    bool isConnected() const;

Q_SIGNALS:
    void latencyUpdate(quint32 latency);
    void dataRateUpdate(quint64 rateUp, quint64 rateDown);
    void mqttConnected();
    void mqttDisconnected();

public Q_SLOTS:
    void logDataUp(quint32 bytes);
    void logDataDown(quint32 bytes);

private:
    QTimer _pingTimer;
    QTimer _dataRateCalcTimer;
    QQueue<quint64> _pingIdQueue;
    QQueue<qint64> _pingTimeQueue;
    quint64 _nextPing;
    quint16 _nextMqttMsgId;
    QMQTT::Client *_mqtt;
    quint64 _bytesDown;
    quint64 _bytesUp;
    int _dataRateCalcInterval;
    int _pingInterval;
};

} // namespace Soro

#endif // MASTERCONNECTIONSTATUSCONTROLLER_H
