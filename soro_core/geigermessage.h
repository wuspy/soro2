#ifndef GEIGERMESSAGE_H
#define GEIGERMESSAGE_H

#include <QByteArray>

#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT GeigerMessage : public AbstractMessage
{
    GeigerMessage();
    GeigerMessage(const QByteArray& payload);
    operator QByteArray() const override;

    quint32 countsPerMinute;
};

} // namespace Soro

#endif // GEIGERMESSAGE_H
