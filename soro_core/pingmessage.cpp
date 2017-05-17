#include "pingmessage.h"

#include <QDataStream>

namespace Soro {

PingMessage::PingMessage() { }

PingMessage::PingMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    stream >> pingId;
}

PingMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << pingId;

    return payload;
}

} // namespace Soro
