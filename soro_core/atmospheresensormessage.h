#ifndef ATMOSPHERESENSORMESSAGE_H
#define ATMOSPHERESENSORMESSAGE_H

#include <QByteArray>

#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT AtmosphereSensorMessage : public AbstractMessage
{
    AtmosphereSensorMessage();
    AtmosphereSensorMessage(const QByteArray& payload);
    operator QByteArray() const override;

    double temperature;
    double humidity;
    quint16 mq2Reading, mq4Reading, mq5Reading, mq6Reading, mq7Reading, mq9Reading, mq135Reading;
    double oxygenPercent;
    quint32 co2Ppm;
    double dustConcentration;
    double windSpeed;
    double windDirection;
};

} // namespace Soro

#endif // ATMOSPHERESENSORMESSAGE_H
