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
    switch (message->type()) {
    case QGst::MessageEos:
        Q_EMIT eos(_id);
        break;
    case QGst::MessageError: {
        QString msg = message.staticCast<QGst::ErrorMessage>()->debugMessage();
        Logger::logError(LogTag, "onBusMessage(): Received error message from gstreamer '" + msg + "'");
        Q_EMIT error(msg, _id);
        break;
    }
    case QGst::MessageInfo: {
        QString msg = message.staticCast<QGst::InfoMessage>()->debugMessage();
        Logger::logInfo(LogTag, "onBusMessage(): Received info message from gstreamer '" + msg + "'");
        break;
    }
    default:
        break;
    }
}

} // namespace Soro
