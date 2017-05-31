#include "sciencecameracontrolsystem.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"
#include "soro_core/sciencecameragimbalmessage.h"

#define LogTag "ScienceCameraControlSystem"

inline qint16 floatToShort(float value)
{
    return (qint16)(value * 32766);
}

namespace Soro {

ScienceCameraControlSystem::ScienceCameraControlSystem(const SettingsModel *settings, QObject *parent) : QObject(parent)
{
    _enabled = false;
    _gamepadLeftX = 0;
    _gamepadLeftY = 0;
    _gamepadRightX = 0;
    _gamepadRightY = 0;

    _mode = settings->getCameraGimbalInputMode();

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    connect(_mqtt, &QMQTT::Client::connected, this, [this]()
    {
        Logger::logInfo(LogTag, "Connected to MQTT broker");
        _mqtt->subscribe("system_down", 2);
    });
    connect(_mqtt, &QMQTT::Client::disconnected, this, [this]()
    {
        Logger::logInfo(LogTag, "Disconnected from MQTT broker");
    });
    _mqtt->setClientId("science_camera_control_system");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(2);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    connect(_mqtt, &QMQTT::Client::received, this, [this](const QMQTT::Message& message)
    {
        if (message.topic() == "system_down")
        {
            QString client = QString(message.payload());
            if (client == "science_package")
            {
                Q_EMIT sciencePackageDisconnected();
            }
            else if (client == "science_package_controller")
            {
                Q_EMIT sciencePackageControllerDisconnected();
            }
            else if (client == "flir")
            {
                Q_EMIT flirDisconnected();
            }
            else if (client == "lidar")
            {
                Q_EMIT lidarDisconnected();
            }
        }
    });

    connect(&_timer, &QTimer::timeout, this, [this]()
    {
        if(_enabled && _mqtt->isConnectedToHost())
        {
            ScienceCameraGimbalMessage msg;

            switch (_mode)
            {
            case SettingsModel::CameraGimbalInputMode_LeftStick:
                msg.xMove = floatToShort(_gamepadLeftX);
                msg.yMove = floatToShort(_gamepadLeftY);
                break;
            case SettingsModel::CameraGimbalInputMode_LeftStickYInverted:
                msg.xMove = floatToShort(_gamepadLeftX);
                msg.yMove = floatToShort(-_gamepadLeftY);
                break;
            case SettingsModel::CameraGimbalInputMode_RightStick:
                msg.xMove = floatToShort(_gamepadRightX);
                msg.yMove = floatToShort(_gamepadRightY);
                break;
            case SettingsModel::CameraGimbalInputMode_RightStickYInverted:
                msg.xMove = floatToShort(_gamepadRightX);
                msg.yMove = floatToShort(-_gamepadRightY);
                break;
            case SettingsModel::CameraGimbalInputMode_BillsWay:
                msg.xMove = floatToShort(_gamepadLeftX);
                msg.yMove = floatToShort(-_gamepadRightY);
                break;
            }
            _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "science_camera_gimbal", msg, 0));
        }
    });

    _timer.start(settings->getCameraGimbalSendInterval());
}

SettingsModel::CameraGimbalInputMode ScienceCameraControlSystem::getInputMode() const
{
    return _mode;
}

void ScienceCameraControlSystem::setInputMode(SettingsModel::CameraGimbalInputMode mode)
{
    _mode = mode;
}

void ScienceCameraControlSystem::enable()
{
    _enabled = true;
}

void ScienceCameraControlSystem::disable()
{
    _enabled = false;
}

void ScienceCameraControlSystem::onGamepadAxisUpdate(SDL_GameControllerAxis axis, float value)
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
