#ifndef BROADCASTER_H
#define BROADCASTER_H

#include <QObject>
#include <QTimerEvent>
#include <QUdpSocket>

namespace Soro {

class Broadcaster : public QObject
{
    Q_OBJECT
public:
    explicit Broadcaster(QObject *parent = 0);

protected:
    void timerEvent(QTimerEvent *e);

private:
    QUdpSocket *_socket;
    int _broadcastTimerId;
};

} // namespace Soro

#endif // BROADCASTER_H
