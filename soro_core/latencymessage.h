#ifndef LATENCYMESSAGE_H
#define LATENCYMESSAGE_H

#include <QByteArray>

#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT LatencyMessage : public AbstractMessage
{
    LatencyMessage();
    LatencyMessage(const QByteArray& payload);
    operator QByteArray() const override;

    quint16 latency;
};

} // namespace Soro

#endif // LATENCYMESSAGE_H
