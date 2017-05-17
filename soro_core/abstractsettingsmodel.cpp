/*
 * Copyright 2017 Jacob Jordan <doublejinitials@ou.edu>
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
#include "logger.h"

#include <QCoreApplication>
#include <QFile>

#define LogTag "AbstractSettingsModel"

namespace Soro {

void AbstractSettingsModel::load()
{
    QHash<QString, int> keys = getKeys();
    for (QString key : keys.keys())
    {
        QVariant value;
        if (getenv(key.toLatin1().constData()) == nullptr)
        {
            // Key is not defined, use default value
            value = getDefaultValues().value(key);
            Logger::logWarn(LogTag, QString("'%1' was not defined in the environment, using default value '%2'").arg(key, value.toString()));
        }
        else
        {
            value = QVariant(getenv(key.toLatin1().constData()));
            Logger::logInfo(LogTag, QString("Using environment value '%2' for setting '%1'").arg(key, value.toString()));
        }

        if (!value.canConvert(keys.value(key)))
        {
            throw QString("Entry for %1 is of the wrong type. It should be '%2', but it appears to be '%3'.")
                    .arg(key, QVariant::typeToName(keys.value(key)), _values.value(key).typeName());
        }
        _values.insert(key, value);
    }
}

} // namespace Soro
