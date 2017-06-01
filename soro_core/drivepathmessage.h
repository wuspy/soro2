#ifndef DRIVEPATHMESSAGE_H
#define DRIVEPATHMESSAGE_H

#include <QByteArray>
#include <QVector>

#include "latlng.h"
#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT DrivePathMessage : public AbstractMessage
{
    DrivePathMessage();
    DrivePathMessage(const QByteArray& payload);
    operator QByteArray() const override;

    QVector<LatLng> points;
};

} // namespace Soro

#endif // DRIVEPATHMESSAGE_H
