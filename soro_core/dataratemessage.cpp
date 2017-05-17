#include "dataratemessage.h"

#include <QDataStream>

namespace Soro {

DataRateMessage::DataRateMessage() { }

DataRateMessage::DataRateMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    stream >> dataRateUp;
    stream >> dataRateDown;
}

DataRateMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << dataRateUp
           << dataRateDown;

    return payload;
}

} // namespace Soro
