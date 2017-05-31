#ifndef SWITCHMESSAGE_H
#define SWITCHMESSAGE_H

#include <QByteArray>

#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT SwitchMessage : public AbstractMessage
{
    SwitchMessage();
    SwitchMessage(const QByteArray& payload);
    operator QByteArray() const override;

    bool on;
};

} // namespace Soro

#endif // SWITCHMESSAGE_H
