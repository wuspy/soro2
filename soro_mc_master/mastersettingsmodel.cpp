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

#include "mastersettingsmodel.h"
#include "libsoromc/constants.h"
#include "maincontroller.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>

#define KEY_PING_INTERVAL "PingInterval"
#define KEY_BITRATE_INTERVAL "BitrateInterval"

#define FILE_PAH SORO_SETTINGS_DIR + "/master.conf"

#define LogTag "MasterSettingsModel"

namespace Soro {

MasterSettingsModel::MasterSettingsModel() {
    _settings = new QSettings(FILE_PAH, QSettings::IniFormat);
}

MasterSettingsModel::~MasterSettingsModel() {
    if (_settings) {
        _settings->sync();
        delete _settings;
    }
}

void MasterSettingsModel::load()
{
    // Check that the settings file exists
    if (!QFile(FILE_PAH).exists())
    {
        MainController::panic(LogTag, QString("The settings file %1 does not exist.").arg(FILE_PAH));
    }
    _settings->sync();

    // Check for any errors loading the settings file
    switch (_settings->status())
    {
    case QSettings::AccessError:
        MainController::panic(LogTag, QString("The settings file %1 could not be accessed. This could mean it's in use by another program.").arg(FILE_PAH));
    case QSettings::FormatError:
        MainController::panic(LogTag, QString("The settings file %1 is malformed.").arg(FILE_PAH));
    default: break;
    }

    if (!_settings->contains(KEY_PING_INTERVAL)) {
        MainController::panic(LogTag, QString("Entry for '%1' was not found in the settings file.").arg(KEY_PING_INTERVAL));
    }
    if (!_settings->contains(KEY_BITRATE_INTERVAL)) {
        MainController::panic(LogTag, QString("Entry for '%1' was not found in the settings file.").arg(KEY_BITRATE_INTERVAL));
    }
}

uint MasterSettingsModel::getPingInterval() const
{
    return _settings->value(KEY_PING_INTERVAL).toUInt();
}

uint MasterSettingsModel::getBitrateInterval() const
{
    return _settings->value(KEY_BITRATE_INTERVAL).toUInt();
}

} // namespace Soro
