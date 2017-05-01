#ifndef BROADCASTRECEIVER_H
#define BROADCASTRECEIVER_H

#include <QObject>
#include <QUdpSocket>

namespace Soro {

class BroadcastReceiver : public QObject
{
    Q_OBJECT
public:
    explicit BroadcastReceiver(quint16 port, int maxMessageLength, QObject *parent = 0);
    ~BroadcastReceiver();

Q_SIGNALS:
    void broadcastReceived(QString message, QHostAddress address, quint16 port);

private:
    int _maxMessageLength;
    char *_buffer;
    QUdpSocket _socket;
};

} // namespace Soro

#endif // BROADCASTRECEIVER_H
