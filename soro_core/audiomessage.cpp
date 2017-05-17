#include "audiomessage.h"

#include <QDataStream>

namespace Soro {

AudioMessage::AudioMessage() { }

AudioMessage::AudioMessage(const QByteArray& payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);
    QString profileStr;

    stream >> profileStr;

    profile = GStreamerUtil::AudioProfile(profileStr);
}

AudioMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << profile.toString();
    return payload;
}

} // namespace Soro
