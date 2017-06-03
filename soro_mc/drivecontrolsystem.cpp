/*
 * Copyright 2017 Stephen Nickle <stephen.nickle@ou.edu>
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

#include "drivecontrolsystem.h"
#include "maincontroller.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"
#include "soro_core/drivemessage.h"
#include "soro_core/switchmessage.h"
#include <QtMath>

#define LogTag "DriveControlSystem"

namespace Soro {

inline float clampF(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

inline qint16 floatToShort(float value)
{
    return (qint16)(value * 32766);
}

DriveControlSystem::DriveControlSystem(const SettingsModel *settings, QObject *parent) : QObject(parent)
{
    _enabled = false;
    _gamepadLeftX = 0;
    _gamepadLeftY = 0;
    _gamepadRightX = 0;
    _gamepadRightY = 0;
    _unfolding = false;
    _mode = settings->getDriveInputMode();

    setLimit(settings->getDrivePowerLimit());

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    connect(_mqtt, &QMQTT::Client::connected, this, [this]()
    {
        Logger::logInfo(LogTag, "Connected to MQTT broker");
        _mqtt->subscribe("system_down", 2);
    });
    connect(_mqtt, &QMQTT::Client::received, this, [this](const QMQTT::Message& msg)
    {
        if (msg.topic() == "system_down")
        {
            QString client = QString(msg.payload());
            if (client == "drive")
            {
                Q_EMIT driveMicrocontrollerDisconnected();
            }
            else if (client == "drive_multiplexer")
            {
                Q_EMIT driveMultiplexerDisconnected();
            }
            else if (client == "drive_controller")
            {
                Q_EMIT driveControllerDisconnected();
            }
        }
    });
    connect(_mqtt, &QMQTT::Client::disconnected, this, [this]()
    {
        Logger::logInfo(LogTag, "Disconnected from MQTT broker");
    });
    _mqtt->setClientId("drive_control_system");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(2);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    connect(&_timer, &QTimer::timeout, this, [this]()
    {
        if(_enabled && _mqtt->isConnectedToHost() && !_unfolding)
        {
            DriveMessage msg;

            switch (_mode)
            {
            case SettingsModel::DriveInputMode_SingleStick: {
                float x = _gamepadLeftX * _limit;
                float y = _gamepadLeftY * _limit;
                float midScale = _skidSteerFactor * (qAbs(x)/1.0f);

                float right, left;

                // First hypotenuse
                float z = sqrt(x*x + y*y);
                // angle in radians
                float rad = z > 0 ? qAcos(qAbs(x)/z) : 0.0f;
                // and in degrees
                float angle = rad*180.0f/3.1415926f;

                // Now angle indicates the measure of turn
                // Along a straight line, with an angle o, the turn co-efficient is same
                // this applies for angles between 0-90, with angle 0 the co-eff is -1
                // with angle 45, the co-efficient is 0 and with angle 90, it is 1
                float tcoeff = -1 + (angle / 90.0f) * 2.0f;
                float turn = clampF(tcoeff * qAbs(qAbs(y) - qAbs(x)), -1.0f, 1.0f);

                // And max of y or x is the movement
                float move = clampF(qMax(qAbs(y), qAbs(x)), -1.0f, 1.0f);

                // First and third quadrant
                if(((x >= 0) & (y >= 0)) | ((x < 0) &  (y < 0)))
                {
                    left = move;
                    right = turn;
                }
                else
                {
                    right = move;
                    left = turn;
                }

                // Reverse polarity
                if(y < 0)
                {
                    left = -left;
                    right = -right;
                }

                qint16 leftS = floatToShort(left);
                qint16 rightS = floatToShort(right);

                msg.wheelFL = leftS;
                msg.wheelML = leftS - floatToShort(midScale * left);
                msg.wheelBL = leftS;
                msg.wheelFR = rightS;
                msg.wheelMR = rightS - floatToShort(midScale * right);
                msg.wheelBR = rightS;
            }
                break;
            case SettingsModel::DriveInputMode_TwoStick: {
                float left = _gamepadLeftY * _limit;
                float right = _gamepadRightY * _limit;
                float midScale = _skidSteerFactor * (qAbs(left - right) / 1.0f);

                qint16 leftS = floatToShort(left);
                qint16 rightS = floatToShort(right);

                msg.wheelFL = leftS;
                msg.wheelML = leftS - floatToShort(midScale * left);
                msg.wheelBL = leftS;
                msg.wheelFR = rightS;
                msg.wheelMR = rightS - floatToShort(midScale * right);
                msg.wheelBR = rightS;
            }
                break;
            }

            _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "drive_controller", msg, 0));
        }
    });

    _timer.start(settings->getDriveSendInterval());
}

void DriveControlSystem::unfold()
{
    if (_enabled && _mqtt->isConnectedToHost())
    {
        _unfolding = true;
        DriveMessage msg;
        msg.wheelFL = 5000;
        msg.wheelFR = 5000;
        msg.wheelBL = -5000;
        msg.wheelBR = -5000;
        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "drive_controller", msg, 0));
    }
}

void DriveControlSystem::stop()
{
    if (_enabled && _mqtt->isConnectedToHost())
    {
        _unfolding = false;
        DriveMessage msg;
        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "drive_controller", msg, 0));
    }
}

void DriveControlSystem::setDriveModeManual()
{
    SwitchMessage msg;
    msg.on = false;
    _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "drive_switch", msg, 2, true));
}

void DriveControlSystem::setDriveModeAutonomous()
{
    SwitchMessage msg;
    msg.on = true;
    _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "drive_switch", msg, 2, true));
}

void DriveControlSystem::setAutonomousDrivePath(DrivePathMessage path)
{
    _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "drive_path", path, 2, true));
}

void DriveControlSystem::setSkidSteerFactor(float factor)
{
    _skidSteerFactor = clampF(factor, 0.0f, 1.0f);
}

float DriveControlSystem::getSkidSteerFactor() const
{
    return _skidSteerFactor;
}

void DriveControlSystem::setLimit(float limit)
{
    _limit = clampF(limit, 0.0f, 1.0f);
    LOG_I(LogTag, "Limit changed to " + QString::number(_limit));
}

float DriveControlSystem::getLimit() const
{
    return _limit;
}

void DriveControlSystem::enable()
{
    _enabled = true;
}

void DriveControlSystem::disable()
{
    _enabled = false;
}

void DriveControlSystem::setInputMode(SettingsModel::DriveInputMode mode)
{
    _mode = mode;
}

SettingsModel::DriveInputMode DriveControlSystem::getInputMode() const
{
    return _mode;
}

void DriveControlSystem::onGamepadAxisUpdate(SDL_GameControllerAxis axis, float value)
{
    switch (axis)
    {
    case SDL_CONTROLLER_AXIS_LEFTX:
        _gamepadLeftX = value;
        break;
    case SDL_CONTROLLER_AXIS_LEFTY:
        _gamepadLeftY = -value;
        break;
    case SDL_CONTROLLER_AXIS_RIGHTX:
        _gamepadRightX = value;
        break;
    case SDL_CONTROLLER_AXIS_RIGHTY:
        _gamepadRightY = -value;
        break;
    default: break;
    }
}

} // namespace Soro
