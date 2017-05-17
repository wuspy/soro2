#include "notificationmessage.h"

#include <QDataStream>

namespace Soro {

NotificationMessage::NotificationMessage() { }

NotificationMessage::NotificationMessage(const QByteArray& payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    quint8 tmpLevel;

    stream >> tmpLevel;
    stream >> title;
    stream >> message;

    reinterpret_cast<qint32&>(level) = static_cast<qint32>(tmpLevel);
}

NotificationMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << static_cast<quint8>(reinterpret_cast<const qint32&>(level))
           << title
           << message;

    return payload;
}

} // namespace Soro
