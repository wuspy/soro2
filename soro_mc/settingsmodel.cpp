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
#include "soro_core/logger.h"

#include <QCoreApplication>

#define KEY_CONFIGURATION "SORO_MC_CONFIGURATION"
#define KEY_MQTT_BROKER_IP "SORO_MQTT_BROKER_IP"
#define KEY_DRIVE_SEND_INTERVAL "SORO_DRIVE_SEND_INTERVAL"
#define KEY_CAMERA_GIMBAL_SEND_INTERVAL "SORO_CAMERA_GIMBAL_SEND_INTERVAL"
#define KEY_ENABLE_HWDECODING "SORO_ENABLE_HW_DECODING"
#define KEY_ENABLE_HWRENDERING "SORO_ENABLE_HW_RENDERING"
#define KEY_DRIVE_INPUT_MODE "SORO_DRIVE_INPUT_MODE"
#define KEY_CAMERA_GIMBAL_INPUT_MODE "SORO_CAMERA_GIMBAL_INPUT_MODE"
#define KEY_DRIVE_SKIDSTEER_FACTOR "SORO_DRIVE_SKIDSTEER_FACTOR"
#define KEY_DRIVE_POWER_LIMIT "SORO_DRIVE_POWER_LIMIT"

#define LogTag "SettingsModel"

namespace Soro {

QHash<QString, int> SettingsModel::getKeys() const
{
    QHash<QString, int> keys;
    keys.insert(KEY_CONFIGURATION, QMetaType::QString);
    keys.insert(KEY_DRIVE_SEND_INTERVAL, QMetaType::UInt);
    keys.insert(KEY_CAMERA_GIMBAL_SEND_INTERVAL, QMetaType::UInt);
    keys.insert(KEY_ENABLE_HWDECODING, QMetaType::Bool);
    keys.insert(KEY_ENABLE_HWRENDERING, QMetaType::Bool);
    keys.insert(KEY_DRIVE_POWER_LIMIT, QMetaType::Float);
    keys.insert(KEY_DRIVE_INPUT_MODE, QMetaType::QString);
    keys.insert(KEY_CAMERA_GIMBAL_INPUT_MODE, QMetaType::QString);
    keys.insert(KEY_DRIVE_SKIDSTEER_FACTOR, QMetaType::Float);
    keys.insert(KEY_MQTT_BROKER_IP, QMetaType::QString);
    return keys;
}

QHash<QString, QVariant> SettingsModel::getDefaultValues() const
{
    QHash<QString, QVariant> defaults;
    defaults.insert(KEY_CONFIGURATION, QVariant("observer"));
    defaults.insert(KEY_DRIVE_SEND_INTERVAL, QVariant(50));
    defaults.insert(KEY_CAMERA_GIMBAL_SEND_INTERVAL, QVariant(50));
    defaults.insert(KEY_ENABLE_HWDECODING, QVariant(false));
    defaults.insert(KEY_ENABLE_HWRENDERING, QVariant(true));
    defaults.insert(KEY_DRIVE_POWER_LIMIT, QVariant(0.5f));
    defaults.insert(KEY_DRIVE_SKIDSTEER_FACTOR, QVariant(0.2f));
    defaults.insert(KEY_DRIVE_INPUT_MODE, "twostick");
    defaults.insert(KEY_CAMERA_GIMBAL_INPUT_MODE, "leftstick");
    defaults.insert(KEY_MQTT_BROKER_IP, QVariant("127.0.0.1"));
    return defaults;
}

SettingsModel::Configuration SettingsModel::getConfiguration() const
{
    QString value = _values.value(KEY_CONFIGURATION).toString().toLower();
    if (value == "driver") return DriverConfiguration;
    if (value == "armoperator") return ArmOperatorConfiguration;
    if (value == "cameraoperator") return CameraOperatorConfiguration;
    if (value == "sciencearmoperator") return ScienceArmOperatorConfiguration;
    if (value == "sciencecameraoperator") return ScienceCameraOperatorConfiguration;
    if (value == "observer") return ObserverConfiguration;

    LOG_W(LogTag, QString("Invalid value for '%1' for setting '%2', returning 'observer'").arg(value, KEY_CONFIGURATION));
    return ObserverConfiguration;
}

SettingsModel::DriveInputMode SettingsModel::getDriveInputMode() const
{
    QString value = _values.value(KEY_DRIVE_INPUT_MODE).toString().toLower();
    if (value == "twostick") return DriveInputMode_TwoStick;
    if (value == "singlestick") return DriveInputMode_SingleStick;

    LOG_W(LogTag, QString("Invalid value for '%1' for setting '%2', returning 'twostick'").arg(value, KEY_DRIVE_INPUT_MODE));
    return DriveInputMode_TwoStick;
}

SettingsModel::CameraGimbalInputMode SettingsModel::getCameraGimbalInputMode() const
{
    QString value = _values.value(KEY_CAMERA_GIMBAL_INPUT_MODE).toString().toLower();
    if (value == "leftstick") return CameraGimbalInputMode_LeftStick;
    if (value == "leftstickinverted") return CameraGimbalInputMode_LeftStickYInverted;
    if (value == "rightstick") return CameraGimbalInputMode_RightStick;
    if (value == "rightstickinverted") return CameraGimbalInputMode_RightStickYInverted;
    if (value == "bill") return CameraGimbalInputMode_BillsWay;

    LOG_W(LogTag, QString("Invalid value for '%1' for setting '%2', returning 'leftstick'").arg(value, KEY_CAMERA_GIMBAL_INPUT_MODE));
    return CameraGimbalInputMode_LeftStick;
}

float SettingsModel::getDriveSkidSteerFactor() const
{
    return _values.value(KEY_DRIVE_SKIDSTEER_FACTOR).toFloat();
}

float SettingsModel::getDrivePowerLimit() const
{
    return _values.value(KEY_DRIVE_POWER_LIMIT).toFloat();
}

uint SettingsModel::getDriveSendInterval() const
{
    return _values.value(KEY_DRIVE_SEND_INTERVAL).toUInt();
}

uint SettingsModel::getCameraGimbalSendInterval() const
{
    return _values.value(KEY_CAMERA_GIMBAL_SEND_INTERVAL).toUInt();
}

bool SettingsModel::getEnableHwDecoding() const
{
    return _values.value(KEY_ENABLE_HWDECODING).toBool();
}

QHostAddress SettingsModel::getMqttBrokerAddress() const
{
    return QHostAddress(_values.value(KEY_MQTT_BROKER_IP).toString());
}

bool SettingsModel::getEnableHwRendering() const
{
    return _values.value(KEY_ENABLE_HWRENDERING).toBool();
}

} // namespace Soro
