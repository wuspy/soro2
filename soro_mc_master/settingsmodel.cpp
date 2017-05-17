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

#include "settingsmodel.h"
#include "soro_core/constants.h"
#include "maincontroller.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>

#define KEY_PING_INTERVAL "SORO_PING_INTERVAL"
#define KEY_MQTT_BROKER_IP "SORO_MQTT_BROKER_IP"
#define KEY_DATA_RATE_CALC_INTERVAL "SORO_DATARATE_CALC_INTERVAL"

namespace Soro {

QHash<QString, int> SettingsModel::getKeys() const
{
    QHash<QString, int> keys;
    keys.insert(KEY_PING_INTERVAL, QMetaType::UInt);
    keys.insert(KEY_DATA_RATE_CALC_INTERVAL, QMetaType::UInt);
    keys.insert(KEY_MQTT_BROKER_IP, QMetaType::QString);
    return keys;
}

QHash<QString, QVariant> SettingsModel::getDefaultValues() const
{
    QHash<QString, QVariant> defaults;
    defaults.insert(KEY_PING_INTERVAL, QVariant(500));
    defaults.insert(KEY_DATA_RATE_CALC_INTERVAL, QVariant(1000));
    defaults.insert(KEY_MQTT_BROKER_IP, "127.0.0.1");
    return defaults;
}

uint SettingsModel::getPingInterval() const
{
    return _values.value(KEY_PING_INTERVAL).toUInt();
}

uint SettingsModel::getDataRateCalcInterval() const
{
    return _values.value(KEY_DATA_RATE_CALC_INTERVAL).toUInt();
}

QHostAddress SettingsModel::getMqttBrokerAddress() const
{
    return QHostAddress(_values.value(KEY_MQTT_BROKER_IP).toString());
}

} // namespace Soro
