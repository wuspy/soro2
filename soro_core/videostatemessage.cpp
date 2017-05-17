#include "videostatemessage.h"

#include <QDataStream>

namespace Soro {

VideoStateMessage::VideoStateMessage() { }

VideoStateMessage::VideoStateMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    quint32 size;
    stream >> size;

    for (int i = 0; i < size; ++i)
    {
        QByteArray payload;
        stream >> payload;
        videoStates.append(VideoMessage(payload));
    }
}

VideoStateMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << (quint32)videoStates.size();
    for (VideoMessage videoMsg : videoStates)
    {
        stream << videoMsg;
    }

    return payload;
}

} // namespace Soro
