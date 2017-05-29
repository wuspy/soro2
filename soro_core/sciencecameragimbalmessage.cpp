#include "sciencecameragimbalmessage.h"

#include <QDataStream>

namespace Soro {

ScienceCameraGimbalMessage::ScienceCameraGimbalMessage()
{
    xMove = yMove = 0;
}

ScienceCameraGimbalMessage::ScienceCameraGimbalMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    stream >> xMove;
    stream >> yMove;
}

ScienceCameraGimbalMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << xMove
           << yMove;

    return payload;
}

} // namespace Soro
