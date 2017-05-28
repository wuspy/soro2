#ifndef DRIVECONTROLSYSTEM_H
#define DRIVECONTROLSYSTEM_H

#include <QObject>
#include <QTimer>
#include <SDL2/SDL.h>

#include "settingsmodel.h"
#include "qmqtt/qmqtt.h"

namespace Soro {

class DriveControlSystem : public QObject
{
    Q_OBJECT

public:
    enum InputMode
    {
        InputMode_TwoStick,
        InputMode_SingleStick
    };

    explicit DriveControlSystem(const SettingsModel *settings, QObject *parent = 0);

    void setSkidSteerFactor(float factor);
    float getSkidSteerFactor() const;

    void setInputMode(InputMode mode);
    InputMode getInputMode() const;

    void setLimit(float limit);
    float getLimit() const;

    void setInterval(int interval);
    int getInterval() const;

Q_SIGNALS:
    void driveSystemExited();

public Q_SLOTS:
    void enable();
    void disable();

    void onGamepadAxisUpdate(SDL_GameControllerAxis axis, float value);

private:
    QTimer _timer;
    QMQTT::Client *_mqtt;

    quint16 _nextMqttMsgId;
    float _skidSteerFactor;
    float _limit;
    bool _enabled;
    InputMode _mode;
    float _gamepadLeftX, _gamepadRightX, _gamepadLeftY, _gamepadRightY;

};


}

#endif // DRIVECONTROLSYSTEM_H
