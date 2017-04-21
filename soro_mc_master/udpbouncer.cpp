#include "udpbouncer.h"
#include "maincontroller.h"
#include "libsoromc/logger.h"

#define LogTag "UdpBouncer"

namespace Soro {

UdpBouncer::UdpBouncer(QHostAddress address, quint16 port, QAbstractSocket::BindMode mode, QObject *parent) : QObject(parent)
{
    if (!_socket.bind(address, port, mode))
    {
        MainController::panic(LogTag, "Unable to bind to UDP port " + address.toString() + ":" + QString::number(port));
    }

    connect(&_socket, &QUdpSocket::readyRead, this, [this]()
    {
        while (_socket.hasPendingDatagrams())
        {
            int len = _socket.read(_buffer, 65536);
            for (int i = 0; i < _bounceAddresses.length(); ++i)
            {
                _socket.writeDatagram(_buffer, len, _bounceAddresses.value(i), _bouncePorts.value(i));
            }
            Q_EMIT bitsDown(len);
        }
    });
}

QHostAddress UdpBouncer::getBindAddress() const
{
    return _socket.localAddress();
}

quint16 UdpBouncer::getBindPort() const
{
    return _socket.localPort();
}

void UdpBouncer::addBounceAddress(QHostAddress address, quint16 port)
{
    if ((address == _socket.localAddress()) && (port == _socket.localPort())) {
        Logger::logWarn(LogTag, "Refusing to add bounce to our own socket");
        return;
    }
    removeBounceAddress(address, port);
    _bounceAddresses.append(address);
    _bouncePorts.append(port);
}

void UdpBouncer::removeBounceAddress(QHostAddress address, quint16 port)
{
    for (int i = 0; i < _bounceAddresses.length(); ++i)
    {
        if ((_bounceAddresses[i] == address) && (_bouncePorts[i] == port))
        {
            _bounceAddresses.removeAt(i);
            _bouncePorts.removeAt(i);
        }
    }
}

} // namespace Soro
