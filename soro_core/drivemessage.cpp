#include "drivemessage.h"

#include <QDataStream>

namespace Soro {

DriveMessage::DriveMessage() { }

DriveMessage::DriveMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    stream >> wheelFL;
    stream >> wheelML;
    stream >> wheelBL;
    stream >> wheelFR;
    stream >> wheelMR;
    stream >> wheelBR;
}

DriveMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << wheelFL
           << wheelML
           << wheelBL
           << wheelFR
           << wheelMR
           << wheelBR;

    return payload;
}

} // namespace Soro
