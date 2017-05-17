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

#include "coresettingsmodel.h"

#include <QCoreApplication>

#define KEY_MQTT_BROKER_ADDRESS "SORO_NET_MQTT_BROKER_ADDRESS"
#define KEY_MEDIA_SERVER_ADDRESS "SORO_NET_MEDIA_SERVER_ADDRESS"
#define KEY_GIMBAL_SYSTEM_PORT "SORO_NET_GIMBAL_SYSTEM_PORT"
#define KEY_DRIVE_SYSTEM_PORT "SORO_NET_DRIVE_SYSTEM_PORT"
#define KEY_ARM_SYSTEM_PORT "SORO_NET_ARM_SYSTEM_PORT"
#define KEY_MQTT_BROKER_PORT "SORO_NET_MQTT_BROKER_PORT"
#define KEY_SERVER_AUDIO_PORT "SORO_NET_SERVER_AUDIO_PORT"
#define KEY_SERVER_FIRST_VIDEO_PORT "SORO_NET_SERVER_FIRST_VIDEO_PORT"
#define KEY_MAX_CAMERAS "SORO_MAX_CAMERAS"
#define KEY_SETTINGS_DIR_PATH "SORO_SETTINGS_DIR"
#define KEY_USE_H264_VAAPI_ENCODE "SORO_GST_USE_H264_VAAPI_ENCODE"
#define KEY_USE_VP8_VAAPI_ENCODE "SORO_GST_USE_VP8_VAAPI_ENCODE"
#define KEY_USE_MPEG2_VAAPI_ENCODE "SORO_GST_USE_MPEG2_VAAPI_ENCODE"
#define KEY_USE_H265_VAAPI_ENCODE "SORO_GST_USE_H265_VAAPI_ENCODE"
#define KEY_USE_JPEG_VAAPI_ENCODE "SORO_GST_USE_JPEG_VAAPI_ENCODE"
#define KEY_USE_H264_VAAPI_DECODE "SORO_GST_USE_H264_VAAPI_DECODE"
#define KEY_USE_VP8_VAAPI_DECODE "SORO_GST_USE_VP8_VAAPI_DECODE"
#define KEY_USE_MPEG2_VAAPI_DECODE "SORO_GST_USE_MPEG2_VAAPI_DECODE"
#define KEY_USE_H265_VAAPI_DECODE "SORO_GST_USE_H265_VAAPI_DECODE"
#define KEY_USE_JPEG_VAAPI_DECODE "SORO_GST_USE_JPEG_VAAPI_DECODE"

#define LogTag "SettingsModel"

namespace Soro {

QHash<QString, int> CoreSettingsModel::getKeys() const
{
    QHash<QString, int> keys;
    keys.insert(KEY_MQTT_BROKER_ADDRESS, QMetaType::QString);
    keys.insert(KEY_MEDIA_SERVER_ADDRESS, QMetaType::QString);
    keys.insert(KEY_GIMBAL_SYSTEM_PORT, QMetaType::UInt);
    keys.insert(KEY_DRIVE_SYSTEM_PORT, QMetaType::UInt);
    keys.insert(KEY_ARM_SYSTEM_PORT, QMetaType::UInt);
    keys.insert(KEY_SERVER_AUDIO_PORT, QMetaType::UInt);
    keys.insert(KEY_MQTT_BROKER_PORT, QMetaType::UInt);
    keys.insert(KEY_SERVER_FIRST_VIDEO_PORT, QMetaType::UInt);
    keys.insert(KEY_MAX_CAMERAS, QMetaType::UInt);
    keys.insert(KEY_SETTINGS_DIR_PATH, QMetaType::QString);
    keys.insert(KEY_USE_H264_VAAPI_ENCODE, QMetaType::Bool);
    keys.insert(KEY_USE_VP8_VAAPI_ENCODE, QMetaType::Bool);
    keys.insert(KEY_USE_MPEG2_VAAPI_ENCODE, QMetaType::Bool);
    keys.insert(KEY_USE_H265_VAAPI_ENCODE, QMetaType::Bool);
    keys.insert(KEY_USE_JPEG_VAAPI_ENCODE, QMetaType::Bool);
    keys.insert(KEY_USE_H264_VAAPI_DECODE, QMetaType::Bool);
    keys.insert(KEY_USE_VP8_VAAPI_DECODE, QMetaType::Bool);
    keys.insert(KEY_USE_MPEG2_VAAPI_DECODE, QMetaType::Bool);
    keys.insert(KEY_USE_H265_VAAPI_DECODE, QMetaType::Bool);
    keys.insert(KEY_USE_JPEG_VAAPI_DECODE, QMetaType::Bool);
    return keys;
}

QHash<QString, QVariant> CoreSettingsModel::getDefaultValues() const
{
    QHash<QString, QVariant> defaults;
    defaults.insert(KEY_MQTT_BROKER_ADDRESS, QVariant("127.0.0.1"));
    defaults.insert(KEY_MEDIA_SERVER_ADDRESS, QVariant("127.0.0.1"));
    defaults.insert(KEY_GIMBAL_SYSTEM_PORT, QVariant(5553));
    defaults.insert(KEY_DRIVE_SYSTEM_PORT, QVariant(5556));
    defaults.insert(KEY_ARM_SYSTEM_PORT, QVariant(5554));
    defaults.insert(KEY_SERVER_AUDIO_PORT, QVariant(5559));
    defaults.insert(KEY_MQTT_BROKER_PORT, QVariant(5557));
    defaults.insert(KEY_MAX_CAMERAS, QVariant(100));
    defaults.insert(KEY_SETTINGS_DIR_PATH, QVariant(QCoreApplication::applicationDirPath() + "/../config"));
    defaults.insert(KEY_USE_H264_VAAPI_ENCODE, QVariant(false));
    defaults.insert(KEY_USE_VP8_VAAPI_ENCODE, QVariant(false));
    defaults.insert(KEY_USE_MPEG2_VAAPI_ENCODE, QVariant(false));
    defaults.insert(KEY_USE_H265_VAAPI_ENCODE, QVariant(false));
    defaults.insert(KEY_USE_JPEG_VAAPI_ENCODE, QVariant(false));
    defaults.insert(KEY_USE_H264_VAAPI_DECODE, QVariant(false));
    defaults.insert(KEY_USE_VP8_VAAPI_DECODE, QVariant(false));
    defaults.insert(KEY_USE_MPEG2_VAAPI_DECODE, QVariant(false));
    defaults.insert(KEY_USE_H265_VAAPI_DECODE, QVariant(false));
    defaults.insert(KEY_USE_JPEG_VAAPI_DECODE, QVariant(false));
    return defaults;
}

QHostAddress CoreSettingsModel::getMqttBrokerAddress() const
{
    return QHostAddress(_values.value(KEY_MQTT_BROKER_ADDRESS).toString());
}

QHostAddress CoreSettingsModel::getMediaServerAddress() const
{
    return QHostAddress(_values.value(KEY_MEDIA_SERVER_ADDRESS).toString());
}

quint16 CoreSettingsModel::getGimbalSystemPort() const
{
    return _values.value(KEY_GIMBAL_SYSTEM_PORT).toUInt();
}

quint16 CoreSettingsModel::getDriveSystemPort() const
{
    return _values.value(KEY_DRIVE_SYSTEM_PORT).toUInt();
}

quint16 CoreSettingsModel::getArmSystemPort() const
{
    return _values.value(KEY_ARM_SYSTEM_PORT).toUInt();
}

quint16 CoreSettingsModel::getMqttBrokerPort() const
{
    return _values.value(KEY_MQTT_BROKER_PORT).toUInt();
}

quint16 CoreSettingsModel::getServerFirstVideoPort() const
{
    return _values.value(KEY_SERVER_FIRST_VIDEO_PORT).toUInt();
}

quint16 CoreSettingsModel::getServerAudioPort() const
{
    return _values.value(KEY_SERVER_AUDIO_PORT).toUInt();
}

uint CoreSettingsModel::getMaxCameras() const
{
    return _values.value(KEY_MAX_CAMERAS).toUInt();
}

QString CoreSettingsModel::getSettingsDirPath() const
{
    return _values.value(KEY_SETTINGS_DIR_PATH).toString();
}

bool CoreSettingsModel::getUseH264VaapiEncode() const
{
    return _values.value(KEY_USE_H264_VAAPI_ENCODE).toBool();
}

bool CoreSettingsModel::getUseVP8VaapiEncode() const
{
    return _values.value(KEY_USE_VP8_VAAPI_ENCODE).toBool();
}

bool CoreSettingsModel::getUseMpeg2VaapiEncode() const
{
    return _values.value(KEY_USE_MPEG2_VAAPI_ENCODE).toBool();
}

bool CoreSettingsModel::getUseH265VaapiEncode() const
{
    return _values.value(KEY_USE_H265_VAAPI_ENCODE).toBool();
}

bool CoreSettingsModel::getUseJpegVaapiEncode() const
{
    return _values.value(KEY_USE_JPEG_VAAPI_ENCODE).toBool();
}

bool CoreSettingsModel::getUseH264VaapiDecode() const
{
    return _values.value(KEY_USE_H264_VAAPI_DECODE).toBool();
}

bool CoreSettingsModel::getUseVP8VaapiDecode() const
{
    return _values.value(KEY_USE_VP8_VAAPI_DECODE).toBool();
}

bool CoreSettingsModel::getUseMpeg2VaapiDecode() const
{
    return _values.value(KEY_USE_MPEG2_VAAPI_DECODE).toBool();
}

bool CoreSettingsModel::getUseH265VaapiDecode() const
{
    return _values.value(KEY_USE_H265_VAAPI_DECODE).toBool();
}

bool CoreSettingsModel::getUseJpegVaapiDecode() const
{
    return _values.value(KEY_USE_JPEG_VAAPI_DECODE).toBool();
}

} // namespace Soro
