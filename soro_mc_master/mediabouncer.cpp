#include "mediabouncer.h"
#include "libsoromc/constants.h"

namespace Soro {

MediaBouncer::MediaBouncer(QObject *parent) : QObject(parent)
{
    _bouncers.append(new UdpBouncer(QHostAddress::Any, SORO_NET_AUDIO_PORT, QAbstractSocket::ShareAddress, this));
    for (quint16 i = SORO_NET_FIRST_VIDEO_PORT; i <= SORO_NET_LAST_VIDEO_PORT; ++i)
    {
        UdpBouncer *b = new UdpBouncer(QHostAddress::Any, i, QAbstractSocket::ShareAddress, this);
        _bouncers.append(b);
        connect(b, &UdpBouncer::bitsRead, this, &MediaBouncer::bitsRead);
    }
}

void MediaBouncer::addBounceAddress(QHostAddress address)
{
    for (UdpBouncer *bouncer : _bouncers)
    {
        bouncer->addBounceAddress(address, bouncer->getBindPort());
    }
}

void MediaBouncer::removeBounceAddress(QHostAddress address)
{
    for (UdpBouncer *bouncer : _bouncers)
    {
        bouncer->removeBounceAddress(address, bouncer->getBindPort());
    }
}

MediaBouncer::~MediaBouncer()
{
    for (quint16 i = 0; i < _bouncers.size(); ++i)
    {
        delete _bouncers[i];
    }
}

} // namespace Soro
