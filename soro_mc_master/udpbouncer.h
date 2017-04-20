#ifndef UDPBOUNCER_H
#define UDPBOUNCER_H

#include <QObject>
#include <QUdpSocket>
#include <QVector>

namespace Soro {

/* Class to forward UDP traffic on a port to any number of other addresses
 */
class UdpBouncer : public QObject
{
    Q_OBJECT
public:
    explicit UdpBouncer(QHostAddress address, quint16 port, QAbstractSocket::BindMode mode=QAbstractSocket::DefaultForPlatform, QObject *parent = 0);

    QHostAddress getBindAddress() const;
    quint16 getBindPort() const;

Q_SIGNALS:
    void bitsRead(int bits);

public Q_SLOTS:
    /* Add an address to forward traffic to
     */
    void addBounceAddress(QHostAddress address, quint16 port);
    /* Remove a previously added address to forward traffic to
     */
    void removeBounceAddress(QHostAddress address, quint16 port);

private:
    char _buffer[65536];
    QUdpSocket _socket;
    QVector<QHostAddress> _bounceAddresses;
    QVector<quint16> _bouncePorts;
};

} // namespace Soro

#endif // UDPBOUNCER_H
