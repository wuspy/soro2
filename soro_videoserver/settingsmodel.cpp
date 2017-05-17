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
#include "maincontroller.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>

#define KEY_COMPUTER_INDEX "SORO_COMPUTER_INDEX"
#define KEY_MQTT_BROKER_IP "SORO_MQTT_BROKER_IP"
#define KEY_USE_H264_VAAPI "SORO_GST_USE_H264_VAAPI"
#define KEY_USE_VP8_VAAPI "SORO_GST_USE_VP8_VAAPI"
#define KEY_USE_MPEG2_VAAPI "SORO_GST_USE_MPEG2_VAAPI"
#define KEY_USE_H265_VAAPI "SORO_GST_USE_H265_VAAPI"
#define KEY_USE_JPEG_VAAPI "SORO_GST_USE_JPEG_VAAPI"

namespace Soro {

QHash<QString, int> SettingsModel::getKeys() const
{
    QHash<QString, int> keys;
    keys.insert(KEY_COMPUTER_INDEX, QMetaType::UInt);
    keys.insert(KEY_USE_H264_VAAPI, QMetaType::Bool);
    keys.insert(KEY_USE_MPEG2_VAAPI, QMetaType::Bool);
    keys.insert(KEY_USE_H265_VAAPI, QMetaType::Bool);
    keys.insert(KEY_USE_JPEG_VAAPI, QMetaType::Bool);
    keys.insert(KEY_MQTT_BROKER_IP, QMetaType::QString);
    return keys;
}

QHash<QString, QVariant> SettingsModel::getDefaultValues() const
{
    QHash<QString, QVariant> defaults;
    defaults.insert(KEY_COMPUTER_INDEX, QVariant(0));
    defaults.insert(KEY_USE_H264_VAAPI, QVariant(false));
    defaults.insert(KEY_USE_VP8_VAAPI, QVariant(false));
    defaults.insert(KEY_USE_MPEG2_VAAPI, QVariant(false));
    defaults.insert(KEY_USE_H265_VAAPI, QVariant(false));
    defaults.insert(KEY_USE_JPEG_VAAPI, QVariant(false));
    defaults.insert(KEY_MQTT_BROKER_IP, QVariant("127.0.0.1"));
    return defaults;
}

uint SettingsModel::getComputerIndex() const
{
    return _values.value(KEY_COMPUTER_INDEX).toUInt();
}

bool SettingsModel::getUseVP8Vaapi() const
{
    return _values.value(KEY_USE_VP8_VAAPI).toBool();
}

bool SettingsModel::getUseMpeg2Vaapi() const
{
    return _values.value(KEY_USE_MPEG2_VAAPI).toBool();
}

bool SettingsModel::getUseH265Vaapi() const
{
    return _values.value(KEY_USE_H265_VAAPI).toBool();
}

bool SettingsModel::getUseH264Vaapi() const
{
    return _values.value(KEY_USE_H264_VAAPI).toBool();
}

bool SettingsModel::getUseJpegVaapi() const
{
    return _values.value(KEY_USE_JPEG_VAAPI).toBool();
}

QHostAddress SettingsModel::getMqttBrokerAddress() const
{
    return QHostAddress(_values.value(KEY_MQTT_BROKER_IP).toString());
}

} // namespace Soro
