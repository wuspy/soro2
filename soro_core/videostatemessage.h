#ifndef VIDEOSTATEMESSAGE_H
#define VIDEOSTATEMESSAGE_H

#include <QByteArray>
#include <QList>

#include "abstractmessage.h"
#include "videomessage.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT VideoStateMessage : public AbstractMessage
{
    VideoStateMessage();
    VideoStateMessage(const QByteArray& payload);
    operator QByteArray() const override;

    QList<VideoMessage> videoStates;
};

} // namespace Soro

#endif // VIDEOSTATEMESSAGE_H
