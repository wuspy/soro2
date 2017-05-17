#ifndef PINGMESSAGE_H
#define PINGMESSAGE_H

#include <QByteArray>

#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT PingMessage : public AbstractMessage
{
    PingMessage();
    PingMessage(const QByteArray& payload);
    operator QByteArray() const override;

    quint64 pingId;
};

} // namespace Soro

#endif // PINGMESSAGE_H
