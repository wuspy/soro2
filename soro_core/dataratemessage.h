#ifndef DATARATEMESSAGE_H
#define DATARATEMESSAGE_H

#include <QByteArray>

#include "abstractmessage.h"

namespace Soro {

struct DataRateMessage : public AbstractMessage
{
    DataRateMessage();
    DataRateMessage(const QByteArray& payload);
    operator QByteArray() const override;

    quint64 dataRateUp;
    quint64 dataRateDown;
};

} // namespace Soro

#endif // DATARATEMESSAGE_H
