#include "gpsmessage.h"

#include <QDataStream>

namespace Soro {

GpsMessage::GpsMessage()
{
    latitude = 0;
    longitude = 0;
    elevation = 0;
}

GpsMessage::GpsMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    stream >> latitude;
    stream >> longitude;
    stream >> elevation;
    stream >> satellites;
}

GpsMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << latitude
           << longitude
           << elevation
           << satellites;

    return payload;
}

} // namespace Soro
