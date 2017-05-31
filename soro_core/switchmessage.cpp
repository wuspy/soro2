#include "switchmessage.h"

#include <QDataStream>

namespace Soro {

SwitchMessage::SwitchMessage()
{
    on = false;
}

SwitchMessage::SwitchMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    quint8 state;
    stream >> state;
    on = state == 1;
}

SwitchMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << (quint8)(on ? 1 : 0);

    return payload;
}

} // namespace Soro
