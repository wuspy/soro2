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

#include "settingsmodel.h"
#include "libsoromc/constants.h"
#include "maincontroller.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>

#define KEY_PING_INTERVAL "PingInterval"
#define KEY_BITRATE_INTERVAL "BitrateInterval"

#define LogTag "MasterSettingsModel"

namespace Soro {

QString SettingsModel::getFilePath() const
{
    return SORO_SETTINGS_DIR + "/master.conf";
}

QHash<QString, int> SettingsModel::getKeys() const
{
    QHash<QString, int> keys;
    keys.insert(KEY_PING_INTERVAL, QMetaType::UInt);
    keys.insert(KEY_BITRATE_INTERVAL, QMetaType::UInt);
    return keys;
}

uint SettingsModel::getPingInterval() const
{
    return _settings->value(KEY_PING_INTERVAL).toUInt();
}

uint SettingsModel::getBitrateInterval() const
{
    return _settings->value(KEY_BITRATE_INTERVAL).toUInt();
}

} // namespace Soro
