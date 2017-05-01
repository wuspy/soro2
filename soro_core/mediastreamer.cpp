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

#include "mediastreamer.h"
#include "gstreamerutil.h"
#include "logger.h"
#include "constants.h"

#include <sys/types.h>
#include <unistd.h>

#define LogTag "MediaStreamer"

namespace Soro {

MediaStreamer::MediaStreamer(QObject *parent) : QObject(parent)
{
    if (!QDBusConnection::sessionBus().isConnected())
    {
        // Not connected to d-bus
        Logger::logError(LogTag, "Not connected to D-Bus system bus");
        exit(1);
    }

    // Register this class as a D-Bus RPC service so other processes can call our public slots
    QDBusConnection::sessionBus().registerObject("/streamchild-" + QString::number(getpid()), this, QDBusConnection::ExportAllSlots);

    _parentInterface = new QDBusInterface(SORO_DBUS_SERVICE_NAME, "/mediaParent", "", QDBusConnection::sessionBus());
    if (!_parentInterface->isValid())
    {
        // Could not create interface for parent process
        Logger::logError(LogTag, "D-Bus parent interface at /mediaParent is not valid");
        exit(2);
    }
    _parentInterface->call(QDBus::NoBlock, "onChildReady", getpid());
}

MediaStreamer::~MediaStreamer()
{
    stopPrivate(false);
    if (_parentInterface)
    {
        delete _parentInterface;
    }
}

void MediaStreamer::stop()
{
    stopPrivate(true);
}

void MediaStreamer::stopPrivate(bool sendReady)
{
    if (_pipeline)
    {
        QGlib::disconnect(pipeline->bus(), "message", this, &VideoStreamer::onBusMessage);
        _pipeline->setState(QGst::StateNull);
        _pipeline.clear();
        if (sendReady)
        {
            _parentInterface->call(QDBus::NoBlock, "onChildReady", getpid());
        }
    }
}

void MediaStreamer::streamVideo(QString device, QString address, quint16 port, QString profile, bool vaapi)
{
    stopPrivate(false);

    _pipeline = createPipeline();

    // create gstreamer command
    QString binStr = GStreamerUtil::createRtpV4L2EncodeString(device, QHostAddress(address), port, GStreamerUtil::AudioProfile(profile), vaapi);
    QGst::BinPtr encoder = QGst::Bin::fromDescription(binStr);

    _pipeline->add(encoder);
    _pipeline->setState(QGst::StatePlaying);

    _parentInterface->call(QDBus::NoBlock, "onChildStreaming", getpid(), address, port, profile, false);
}

void MediaStreamer::streamStereoVideo(QString leftDevice, QString rightDevice, QString address, quint16 port, QString profile, bool vaapi)
{
    stopPrivate(false);

    _pipeline = createPipeline();

    // create gstreamer command
    QString binStr = GStreamerUtil::createRtpStereoV4L2EncodeString(leftDevice, rightDevice, QHostAddress(address), port, GStreamerUtil::AudioProfile(profile), vaapi);
    QGst::BinPtr encoder = QGst::Bin::fromDescription(binStr);

    _pipeline->add(encoder);
    _pipeline->setState(QGst::StatePlaying);

    _parentInterface->call(QDBus::NoBlock, "onChildStreaming", getpid(), address, port, profile, true);
}

QGst::PipelinePtr MediaStreamer::createPipeline()
{
    QGst::PipelinePtr pipeline = QGst::Pipeline::create();
    pipeline->bus()->addSignalWatch();
    QGlib::connect(pipeline->bus(), "message", this, &VideoStreamer::onBusMessage);

    return pipeline;
}

void MediaStreamer::onBusMessage(const QGst::MessagePtr & message)
{
    QByteArray errorMessage;
    switch (message->type())
    {
    case QGst::MessageEos:
        _parentInterface->call(QDBus::NoBlock, "onChildError", getpid(), "Received EOS message from GStreamer");
        stopPrivate(true);
        break;
    case QGst::MessageError:
        errorMessage = message.staticCast<QGst::ErrorMessage>()->error().message().toLatin1();
        _parentInterface->call(QDBus::NoBlock, "onChildError", getpid(), errorMessage);
        stopPrivate(true);
        break;
    default:
        break;
    }
}

} // namespace Soro

