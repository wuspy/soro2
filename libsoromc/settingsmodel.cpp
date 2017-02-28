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

#define KEY_MASTER "Master"

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

bool SettingsModel::load()
{
    // Check that the settings file exists
    if (!QFile(SORO_MC_SETTINGS_FILE).exists())
    {
        _errorString = QString("The settings file %1 does not exist.").arg(SORO_MC_SETTINGS_FILE);
        return false;
    }
    _settings->sync();

    // Check for any errors loading the settings file
    switch (_settings->status())
    {
    case QSettings::AccessError:
        _errorString = QString("The settings file %1 could not be accessed. This could mean it's in use by another program.").arg(SORO_MC_SETTINGS_FILE);
        return false;
    case QSettings::FormatError:
        _errorString = QString("The settings file %1 is malformed.").arg(SORO_MC_SETTINGS_FILE);
        return false;
    default: break;
    }

    if (!_settings->contains(KEY_MASTER))
    {
        _errorString = QString("Entry for '%1' was not found in the settings file.").arg(KEY_MASTER);
        return false;
    }


    return true;
}

bool SettingsModel::write()
{
    // Write the changes to the file
    _settings->sync();

    // Check for errors
    if (_settings->status() != QSettings::NoError)
    {
        _errorString = "Internal QSettings error while writing data";
        return false;
    }
    return true;
}

QString SettingsModel::errorString() const
{
    return _errorString;
}

bool SettingsModel::getIsMaster() const
{
    return _settings->value(KEY_MASTER).toBool();
}

void SettingsModel::setIsMaster(bool master)
{
    _settings->setValue(KEY_MASTER, master);
}

} // namespace Soro
