/*
 * Copyright 2017 The University of Oklahoma.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gstreamerpipelinewatch.h"
#include "soro_core/logger.h"

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
        LOG_E(LogTag, "onBusMessage(): Received EOS message from gstreamer");
        Q_EMIT error("Received unexpected EOS message", _id);
        break;
    case QGst::MessageError: {
        QString msg = message.staticCast<QGst::ErrorMessage>()->debugMessage();
        LOG_E(LogTag, "onBusMessage(): Received error message from gstreamer '" + msg + "'");
        Q_EMIT error(msg, _id);
        break;
    }
    case QGst::MessageInfo: {
        QString msg = message.staticCast<QGst::InfoMessage>()->debugMessage();
        LOG_I(LogTag, "onBusMessage(): Received info message from gstreamer '" + msg + "'");
        break;
    }
    default:
        break;
    }
}

} // namespace Soro
