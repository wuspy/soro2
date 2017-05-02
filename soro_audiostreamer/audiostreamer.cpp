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

#include "audiostreamer.h"
#include "gstreamerutil.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"

#include <Qt5GStreamer/QGlib/Connect>
#include <Qt5GStreamer/QGst/Bus>

#include <sys/types.h>
#include <unistd.h>

#define LogTag "MediaStreamer"

namespace Soro {

AudioStreamer::AudioStreamer(QObject *parent) : QObject(parent)
{
    if (!QDBusConnection::sessionBus().isConnected())
    {
        // Not connected to d-bus
        Logger::logError(LogTag, "Not connected to D-Bus system bus");
        exit(1);
    }

    // Register this class as a D-Bus RPC service so other processes can call our public slots
    QDBusConnection::sessionBus().registerObject("/", this, QDBusConnection::ExportAllSlots);

    _parentInterface = new QDBusInterface(SORO_DBUS_AUDIO_PARENT_SERVICE_NAME, "/", "", QDBusConnection::sessionBus());
    if (!_parentInterface->isValid())
    {
        // Could not create interface for parent process
        Logger::logError(LogTag, "D-Bus parent interface at /audioParent is not valid");
        exit(2);
    }
    _parentInterface->call(QDBus::NoBlock, "onChildReady", getpid());
}

AudioStreamer::~AudioStreamer()
{
    stop();
}

void AudioStreamer::stop()
{
    stopPrivate(true);
}

void AudioStreamer::stopPrivate(bool sendReady)
{
    if (_pipeline)
    {
        QGlib::disconnect(_pipeline->bus(), "message", this, &AudioStreamer::onBusMessage);
        _pipeline->setState(QGst::StateNull);
        _pipeline.clear();
        if (sendReady)
        {
            _parentInterface->call(QDBus::NoBlock, "onChildReady", getpid());
        }
    }
}

void AudioStreamer::streamAudio(QString address, quint16 port, QString profile)
{
    stopPrivate(false);

    _pipeline = QGst::Pipeline::create("audio");

    // create gstreamer command
    QString binStr = GStreamerUtil::createRtpAlsaEncodeString(QHostAddress(address), port, GStreamerUtil::AudioProfile(profile));
    QGst::BinPtr encoder = QGst::Bin::fromDescription(binStr);

    _pipeline->add(encoder);
    _pipeline->setState(QGst::StatePlaying);

    _parentInterface->call(QDBus::NoBlock, "onChildStreaming", getpid(), address, port, profile);
}

QGst::PipelinePtr AudioStreamer::createPipeline()
{
    QGst::PipelinePtr pipeline = QGst::Pipeline::create("audio");
    pipeline->bus()->addSignalWatch();
    QGlib::connect(pipeline->bus(), "message", this, &AudioStreamer::onBusMessage);

    return pipeline;
}

void AudioStreamer::onBusMessage(const QGst::MessagePtr & message)
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

