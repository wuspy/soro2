#ifndef NOTIFICATIONMESSAGE_H
#define NOTIFICATIONMESSAGE_H

#include <QByteArray>
#include <QString>

#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT NotificationMessage : public AbstractMessage
{
    enum Level
    {
        Level_Error = 0,
        Level_Warning,
        Level_Info
    };

    NotificationMessage();
    NotificationMessage(const QByteArray& payload);
    operator QByteArray() const override;

    Level level;
    QString title;
    QString message;
};

} // namespace Soro

#endif // NOTIFICATIONMESSAGE_H
