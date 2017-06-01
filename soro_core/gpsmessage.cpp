#include "gpsmessage.h"

#include <QDataStream>

namespace Soro {

GpsMessage::GpsMessage()
{
    elevation = 0;
}

GpsMessage::GpsMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    stream >> location.latitude;
    stream >> location.longitude;
    stream >> elevation;
    stream >> satellites;
}

GpsMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << location.latitude
           << location.longitude
           << elevation
           << satellites;

    return payload;
}

} // namespace Soro
