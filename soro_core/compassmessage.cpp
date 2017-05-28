#include "compassmessage.h"

#include <QDataStream>

namespace Soro {

CompassMessage::CompassMessage() { }

CompassMessage::CompassMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    stream >> heading;
}

CompassMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << heading;

    return payload;
}

} // namespace Soro
