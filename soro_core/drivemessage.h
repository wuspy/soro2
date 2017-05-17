#ifndef DRIVEMESSAGE_H
#define DRIVEMESSAGE_H

#include <QByteArray>

#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT DriveMessage : public AbstractMessage
{
    DriveMessage();
    DriveMessage(const QByteArray& payload);
    operator QByteArray() const override;

    qint16 wheelFL;
    qint16 wheelML;
    qint16 wheelBL;
    qint16 wheelFR;
    qint16 wheelMR;
    qint16 wheelBR;
};

} // namespace Soro

#endif // DRIVEMESSAGE_H
