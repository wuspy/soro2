#include "broadcaster.h"
#include "libsoromc/constants.h"

namespace Soro {

Broadcaster::Broadcaster(QObject *parent) : QObject(parent)
{
    _socket = new QUdpSocket(this);
    if (!_socket->bind(SORO_MC_MASTER_BROADCAST_PORT))
    {
        throw QString("Could not bind to UDP port %1").arg(SORO_MC_MASTER_BROADCAST_PORT);
    }
    _broadcastTimerId = startTimer(500);
}

void Broadcaster::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _broadcastTimerId)
    {
        _socket->writeDatagram(QByteArray("master"), QHostAddress::Broadcast, SORO_MC_MASTER_BROADCAST_PORT);
    }
}

} // namespace Soro
