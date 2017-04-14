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
#include "constants.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>

#define KEY_CONFIGURATION "Configuration"
#define KEY_ENABLE_HWDECODING "EnableHwDecoding"
#define KEY_ENABLE_HWRENDERING "EnableHwRendering"

namespace Soro {

SettingsModel::SettingsModel() {
    _settings = new QSettings(SORO_MC_SETTINGS_FILE, QSettings::IniFormat);
}

SettingsModel::~SettingsModel() {
    if (_settings) {
        _settings->sync();
        delete _settings;
    }
}

void SettingsModel::load()
{
    // Check that the settings file exists
    if (!QFile(SORO_MC_SETTINGS_FILE).exists())
    {
        throw QString("The settings file %1 does not exist.").arg(SORO_MC_SETTINGS_FILE);
    }
    _settings->sync();

    // Check for any errors loading the settings file
    switch (_settings->status())
    {
    case QSettings::AccessError:
        throw QString("The settings file %1 could not be accessed. This could mean it's in use by another program.").arg(SORO_MC_SETTINGS_FILE);
    case QSettings::FormatError:
        throw QString("The settings file %1 is malformed.").arg(SORO_MC_SETTINGS_FILE);
    default: break;
    }

    if (!_settings->contains(KEY_CONFIGURATION)) {
        throw QString("Entry for '%1' was not found in the settings file.").arg(KEY_CONFIGURATION);
    }
    if (!_settings->contains(KEY_ENABLE_HWDECODING)) {
        throw QString("Entry for '%1' was not found in the settings file.").arg(KEY_ENABLE_HWDECODING);
    }
    if (!_settings->contains(KEY_ENABLE_HWRENDERING)) {
        throw QString("Entry for '%1' was not found in the settings file.").arg(KEY_ENABLE_HWRENDERING);
    }
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

bool SettingsModel::getEnableHwDecoding() const
{
    return _settings->value(KEY_ENABLE_HWDECODING).toBool();
}

bool SettingsModel::getEnableHwRendering() const
{
    return _settings->value(KEY_ENABLE_HWRENDERING).toBool();
}

} // namespace Soro
