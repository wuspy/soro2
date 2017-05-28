#ifndef COMPASSMESSAGE_H
#define COMPASSMESSAGE_H

#include <QByteArray>

#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT CompassMessage : public AbstractMessage
{
    CompassMessage();
    CompassMessage(const QByteArray& payload);
    operator QByteArray() const override;

    double heading;
};

} // namespace Soro

#endif // COMPASSMESSAGE_H
