#include "latencymessage.h"

#include <QDataStream>

namespace Soro {

LatencyMessage::LatencyMessage() { }

LatencyMessage::LatencyMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    stream >> latency;
}

LatencyMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << latency;

    return payload;
}

} // namespace Soro
