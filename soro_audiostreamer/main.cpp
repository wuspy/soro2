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

#include <QCoreApplication>
#include <QByteArray>
#include <QDataStream>
#include <QtDBus>

#include <sys/types.h>
#include <unistd.h>

#include <Qt5GStreamer/QGst/Init>

#include "audiostreamer.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"

#define LOG_TAG "Main"

using namespace Soro;

int main(int argc, char *argv[])
{
    QCoreApplication::setOrganizationName("Sooner Rover");
    QCoreApplication::setOrganizationDomain("ou.edu/soonerrover");
    QCoreApplication::setApplicationName("Audio Streamer");
    QCoreApplication app(argc, argv);

    Logger::logInfo(LOG_TAG, "Starting...");
    QGst::init();

    if (!QDBusConnection::sessionBus().isConnected()) {
        Logger::logError(LOG_TAG, "Cannot connect to D-Bus session bus");
        return 1;
    }

    if (!QDBusConnection::sessionBus().registerService(SORO_DBUS_AUDIO_CHILD_SERVICE_NAME(QString::number(getpid())))) {
        Logger::logError(LOG_TAG, "Cannot register D-Bus service: " + QDBusConnection::sessionBus().lastError().message());
        return 1;
    }

    AudioStreamer s;

    return app.exec();
}
