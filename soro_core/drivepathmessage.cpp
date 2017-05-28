#include "drivepathmessage.h"

#include <QDataStream>

namespace Soro {

DrivePathMessage::DrivePathMessage() { }

DrivePathMessage::DrivePathMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    quint16 length;
    stream >> length;

    for (int i = 0; i < length; ++i)
    {
        Location point;
        stream >> point.latitude;
        stream >> point.longitude;
        points.append(point);
    }
}

DrivePathMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << (quint16)points.length();

    for (Location point : points)
    {
        stream << point.latitude
               << point.longitude;
    }

    return payload;
}

} // namespace Soro
