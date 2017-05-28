#include "geigermessage.h"

#include <QDataStream>

namespace Soro {

GeigerMessage::GeigerMessage()
{
    countsPerMinute = 0;
}

GeigerMessage::GeigerMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    stream >> countsPerMinute;
}

GeigerMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << countsPerMinute;

    return payload;
}

} // namespace Soro
