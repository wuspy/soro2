#ifndef ARMMESSAGE_H
#define ARMMESSAGE_H

#include "abstractmessage.h"
#include "soro_core_global.h"

#include <QByteArray>

namespace Soro {

struct SORO_CORE_EXPORT ArmMessage : public AbstractMessage
{
    ArmMessage();
    ArmMessage(const QByteArray& payload);
    operator QByteArray() const override;

    QByteArray masterArmData;
};

} // namespace Soro

#endif // ARMMESSAGE_H
