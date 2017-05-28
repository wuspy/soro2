#include "spectrometermessage.h"

#include <QDataStream>

namespace Soro {

SpectrometerMessage::SpectrometerMessage() { }

SpectrometerMessage::SpectrometerMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    spectrumWhite.clear();
    spectrum404.clear();

    quint16 temp;

    for (int i = 0; i < SPECTRUM_POINTS; ++i)
    {
        stream >> temp;
        spectrumWhite.append(temp);
    }
    for (int i = 0; i < SPECTRUM_POINTS; ++i)
    {
        stream >> temp;
        spectrum404.append(temp);
    }
}

SpectrometerMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    for (int i = 0; i < SPECTRUM_POINTS; ++i)
    {
        stream << spectrumWhite.value(i);
    }
    for (int i = 0; i < SPECTRUM_POINTS; ++i)
    {
        stream << spectrum404.value(i);
    }

    return payload;
}

} // namespace Soro
