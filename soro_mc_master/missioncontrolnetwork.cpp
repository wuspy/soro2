#include "missioncontrolnetwork.h"
#include "soro_core/constants.h"
#include "soro_core/logger.h"

namespace Soro {

MissionControlNetwork::MissionControlNetwork(QObject *parent)
    : QObject(parent), _broadcastReceiver(SORO_NET_MC_BROADCAST_PORT, 100, this)
{
    connect (&_broadcastReceiver, &BroadcastReceiver::broadcastReceived, this, [this](QString message, QHostAddress address, quint16 port)
    {

    });
}

} // namespace Soro
