#ifndef SPECTROMETERMESSAGE_H
#define SPECTROMETERMESSAGE_H

#include <QByteArray>
#include <QVector>

#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT SpectrometerMessage : public AbstractMessage
{
    SpectrometerMessage();
    SpectrometerMessage(const QByteArray& payload);
    operator QByteArray() const override;

    static const int SPECTRUM_POINTS = 288;

    QVector<quint16> spectrumWhite;
    QVector<quint16> spectrum404;
};

} // namespace Soro

#endif // SPECTROMETERMESSAGE_H
