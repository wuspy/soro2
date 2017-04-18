#include "gstreamerpipelinewatch.h"
#include "libsoromc/logger.h"

#include <Qt5GStreamer/QGst/Bus>
#include <Qt5GStreamer/QGlib/Connect>

#define LogTag "GStreamerPipelineWatch"

namespace Soro {

GStreamerPipelineWatch::GStreamerPipelineWatch(int id, QGst::PipelinePtr pipeline, QObject *parent) : QObject(parent)
{
    _id = id;
    pipeline->bus()->addSignalWatch();
    QGlib::connect(pipeline->bus(), "message", this, &GStreamerPipelineWatch::onBusMessage);
}

void GStreamerPipelineWatch::onBusMessage(const QGst::MessagePtr &message)
{
    Logger::logInfo(LogTag, "onBusMessage(): Got bus message type " + message->typeName());
    switch (message->type()) {
    case QGst::MessageEos:
        Q_EMIT eos(_id);
        break;
    case QGst::MessageError: {
        QString errorMessage = message.staticCast<QGst::ErrorMessage>()->error().message().toLatin1();
        Logger::logError(LogTag, "onBusMessage(): Received error message from gstreamer '" + errorMessage + "'");
        Q_EMIT error(errorMessage, _id);
        break;
    }
    default:
        break;
    }
}

} // namespace Soro
