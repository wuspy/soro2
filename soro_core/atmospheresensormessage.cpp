#include "atmospheresensormessage.h"

#include <QDataStream>

namespace Soro {

AtmosphereSensorMessage::AtmosphereSensorMessage() { }

AtmosphereSensorMessage::AtmosphereSensorMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);

    stream >> temperature;
    stream >> humidity;
    stream >> mq2Reading;
    stream >> mq4Reading;
    stream >> mq5Reading;
    stream >> mq6Reading;
    stream >> mq7Reading;
    stream >> mq9Reading;
    stream >> mq135Reading;
    stream >> oxygenPercent;
    stream >> co2Ppm;
    stream >> dustConcentration;
    stream >> windSpeed;
    stream >> windDirection;
}

AtmosphereSensorMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << temperature
           << humidity
           << mq2Reading
           << mq4Reading
           << mq5Reading
           << mq6Reading
           << mq7Reading
           << mq9Reading
           << mq135Reading
           << oxygenPercent
           << co2Ppm
           << dustConcentration
           << windSpeed
           << windDirection;

    return payload;
}

} // namespace Soro
