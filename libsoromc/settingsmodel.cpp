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

#include <QCoreApplication>
#include <QFile>

#define SETTINGS_FILE_PATH QString("%1/mc.conf").arg(QCoreApplication::applicationDirPath())

#define KEY_ROS_MASTER_URI "RosMasterUri"

namespace Soro {

SettingsModel::SettingsModel() {
    _settings = new QSettings(SETTINGS_FILE_PATH, QSettings::IniFormat);
}

SettingsModel::~SettingsModel() {
    if (_settings) {
        _settings->sync();
        delete _settings;
    }
}

bool SettingsModel::checkKeyExists(QString key, QString *error)
{
    if (!_settings->contains(key))
    {
        if (error)
        {
            *error = QString("Entry for '%1' was not found in the settings file.").arg(key);
            return false;
        }
    }
    return true;
}

bool SettingsModel::load(QString *error)
{
    // Check that the settings file exists
    if (!QFile(SETTINGS_FILE_PATH).exists())
    {
        if (error)
        {
            *error = QString("The settings file %1 does not exist.").arg(SETTINGS_FILE_PATH);
            return false;
        }
    }
    _settings->sync();

    // Check for any errors loading the settings file
    switch (_settings->status())
    {
    case QSettings::AccessError:
        if (error)
        {
            *error = QString("The settings file %1 could not be accessed. This could mean it's in use by another program.").arg(SETTINGS_FILE_PATH);
        }
        return false;
    case QSettings::FormatError:
        if (error)
        {
            *error = QString("The settings file %1 is malformed.").arg(SETTINGS_FILE_PATH);
        }
        return false;
    default: break;
    }

    if (!checkKeyExists(KEY_ROS_MASTER_URI, error)) return false;

    return true;
}

bool SettingsModel::write(QString *error)
{
    // Write the changes to the file
    _settings->sync();

    // Check for errors
    if (_settings->status() != QSettings::NoError)
    {
        if (error)
        {
            *error = "Internal QSettings error while writing data";
            return false;
        }
    }
    return true;
}

QString SettingsModel::getRosMasterUri() const
{
    return _settings->value(KEY_ROS_MASTER_URI).toString();
}

void SettingsModel::setRosMasterUri(QString uri)
{
    _settings->setValue(KEY_ROS_MASTER_URI, uri);
}

} // namespace Soro
