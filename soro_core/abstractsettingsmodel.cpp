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

#include "abstractsettingsmodel.h"
#include "constants.h"

#include <QCoreApplication>
#include <QFile>

namespace Soro {

AbstractSettingsModel::~AbstractSettingsModel()
{
    if (_settings) delete _settings;
}

void AbstractSettingsModel::fatalLoadError(QString message)
{
    delete _settings;
    _settings = nullptr;
    throw message;
}

void AbstractSettingsModel::load()
{
    _settings = new QSettings(getFilePath(), QSettings::IniFormat);
    // Check that the settings file exists
    if (!QFile(getFilePath()).exists())
    {
        fatalLoadError(QString("The settings file %1 does not exist.").arg(getFilePath()));
    }
    _settings->sync();

    // Check for any errors loading the settings file
    switch (_settings->status())
    {
    case QSettings::AccessError:
        fatalLoadError(QString("The settings file %1 could not be accessed. This could mean it's in use by another program.").arg(getFilePath()));
        break;
    case QSettings::FormatError:
        fatalLoadError(QString("The settings file %1 is malformed.").arg(getFilePath()));
        break;
    default: break;
    }

    QHash<QString, int> keys = getKeys();
    Q_FOREACH (QString key, keys.keys())
    {
        if (!_settings->contains(key))
        {
            fatalLoadError(QString("Entry for '%1' was not found in the settings file.").arg(key));
        }
        if (!_settings->value(key).canConvert(keys.value(key)))
        {
            fatalLoadError(QString("Entry for %1 is of the wrong type. It should be '%2', but it appears to be '%3'.")
                    .arg(key, QVariant::typeToName(keys.value(key)), _settings->value(key).typeName()));
        }
    }
}

} // namespace Soro
