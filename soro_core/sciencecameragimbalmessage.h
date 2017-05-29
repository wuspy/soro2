#ifndef SCIENCECAMERAGIMBALMESSAGE_H
#define SCIENCECAMERAGIMBALMESSAGE_H

#include <QByteArray>

#include "abstractmessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT ScienceCameraGimbalMessage : public AbstractMessage
{
    ScienceCameraGimbalMessage();
    ScienceCameraGimbalMessage(const QByteArray& payload);
    operator QByteArray() const override;

    qint16 xMove, yMove;
};

} // namespace Soro

#endif // SCIENCECAMERAGIMBALMESSAGE_H
