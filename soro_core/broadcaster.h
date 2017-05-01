#ifndef BROADCASTER_H
#define BROADCASTER_H

#include <QObject>
#include <QTimerEvent>
#include <QUdpSocket>

namespace Soro {

/* Class to broadcast a message on a UDP port at a set time interval.
 */
class Broadcaster : public QObject
{
    Q_OBJECT
public:
    explicit Broadcaster(quint16 port, QString message, int interval, QObject *parent = 0);

protected:
    void timerEvent(QTimerEvent *e);

private:
    const char *_message;
    quint16 _port;
    QUdpSocket _socket;
    int _broadcastTimerId;
};

} // namespace Soro

#endif // BROADCASTER_H
