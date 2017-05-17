#ifndef AUDIOMESSAGE_H
#define AUDIOMESSAGE_H

#include <QByteArray>

#include "abstractmessage.h"
#include "gstreamerutil.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT AudioMessage : public AbstractMessage
{
    AudioMessage();
    AudioMessage(const QByteArray& payload);
    operator QByteArray() const override;

    GStreamerUtil::AudioProfile profile;
};

} // namespace Soro

#endif // AUDIOMESSAGE_H
