#include "broadcastreceiver.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"

#define LogTag "BroadcastReceiver"

namespace Soro {

BroadcastReceiver::BroadcastReceiver(quint16 port, int maxMessageLength, QObject *parent) : QObject(parent)
{
    _buffer = new char[maxMessageLength];

    if (!_socket.bind(port))
    {
        throw QString("Cannot bind to UDP port " + QString::number(port));
    }
    connect(&_socket, &QUdpSocket::readyRead, this, [this]()
    {
        while (_socket.hasPendingDatagrams())
        {
            QHostAddress address;
            quint16 port;
            int len = _socket.readDatagram(_buffer, _maxMessageLength, &address, &port);
            if (len >= _maxMessageLength)
            {
                Logger::logWarn(LogTag, "Received message larger than the configured maximum length");
                continue;
            }
            _buffer[_maxMessageLength - 1] = 0;
            Q_EMIT broadcastReceived(QString(_buffer), address, port);
        }
    });
}

BroadcastReceiver::~BroadcastReceiver()
{
    disconnect(&_socket, &QUdpSocket::readyRead, this, 0);
    delete[] _buffer;
}

} // namespace Soro
