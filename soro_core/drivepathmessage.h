#ifndef DRIVEPATHMESSAGE_H
#define DRIVEPATHMESSAGE_H

#include <QByteArray>
#include <QVector>

#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT DrivePathMessage : public AbstractMessage
{
    DrivePathMessage();
    DrivePathMessage(const QByteArray& payload);
    operator QByteArray() const override;

    struct Location
    {
        double latitude;
        double longitude;
    };

    QVector<Location> points;
};

} // namespace Soro

#endif // DRIVEPATHMESSAGE_H
