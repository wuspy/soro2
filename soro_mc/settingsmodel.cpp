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
#include "soro_core/constants.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>

#define KEY_CONFIGURATION "Configuration"
#define KEY_DRIVE_SEND_INTERVAL "DriveSendInterval"
#define KEY_ENABLE_HWDECODING "EnableHwDecoding"
#define KEY_ENABLE_HWRENDERING "EnableHwRendering"

namespace Soro {

QString SettingsModel::getFilePath() const
{
    return SORO_MC_SETTINGS_DIR + "/mc.conf";
}

QHash<QString, int> SettingsModel::getKeys() const
{
    QHash<QString, int> keys;
    keys.insert(KEY_CONFIGURATION, QMetaType::QString);
    keys.insert(KEY_DRIVE_SEND_INTERVAL, QMetaType::UInt);
    keys.insert(KEY_ENABLE_HWDECODING, QMetaType::Bool);
    keys.insert(KEY_ENABLE_HWRENDERING, QMetaType::Bool);
    return keys;
}

SettingsModel::Configuration SettingsModel::getConfiguration() const
{
    QString value = _settings->value(KEY_CONFIGURATION).toString().toLower();
    if (value == "driver") return DriverConfiguration;
    if (value == "armoperator") return ArmOperatorConfiguration;
    if (value == "cameraoperator") return CameraOperatorConfiguration;
    if (value == "observer") return ObserverConfiguration;
    throw QString("Invalid value for 'configuration' key in settings");
}

uint SettingsModel::getDriveSendInterval() const
{
    return _settings->value(KEY_DRIVE_SEND_INTERVAL).toUInt();
}

bool SettingsModel::getEnableHwDecoding() const
{
    return _settings->value(KEY_ENABLE_HWDECODING).toBool();
}

bool SettingsModel::getEnableHwRendering() const
{
    return _settings->value(KEY_ENABLE_HWRENDERING).toBool();
}

} // namespace Soro
