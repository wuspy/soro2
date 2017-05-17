#include "addmediabouncemessage.h"

#include <QDataStream>

namespace Soro {

AddMediaBounceMessage::AddMediaBounceMessage() { }

AddMediaBounceMessage::AddMediaBounceMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    stream >> clientID;
    stream >> address;
}

QByteArray AddMediaBounceMessage::serialize() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << clientID
           << address;

    return payload;
}

} // namespace Soro
