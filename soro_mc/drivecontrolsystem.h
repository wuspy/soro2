#ifndef DRIVECONTROLSYSTEM_H
#define DRIVECONTROLSYSTEM_H

#include <QObject>
#include <QTimer>
#include <SDL2/SDL.h>

#include "settingsmodel.h"
#include "soro_core/drivepathmessage.h"

#include "qmqtt/qmqtt.h"

namespace Soro {

class DriveControlSystem : public QObject
{
    Q_OBJECT

public:
    explicit DriveControlSystem(const SettingsModel *settings, QObject *parent = 0);

    void setSkidSteerFactor(float factor);
    float getSkidSteerFactor() const;

    void setInputMode(SettingsModel::DriveInputMode mode);
    SettingsModel::DriveInputMode getInputMode() const;

    void setLimit(float limit);
    float getLimit() const;

Q_SIGNALS:
    void driveSystemExited();

public Q_SLOTS:
    void enable();
    void disable();
    void setDriveModeManual();
    void setDriveModeAutonomous();
    void setAutonomousDrivePath(DrivePathMessage path);

    void onGamepadAxisUpdate(SDL_GameControllerAxis axis, float value);

private:
    QTimer _timer;
    QMQTT::Client *_mqtt;

    quint16 _nextMqttMsgId;
    float _skidSteerFactor;
    float _limit;
    bool _enabled;
    SettingsModel::DriveInputMode _mode;
    float _gamepadLeftX, _gamepadRightX, _gamepadLeftY, _gamepadRightY;

};


}

#endif // DRIVECONTROLSYSTEM_H
