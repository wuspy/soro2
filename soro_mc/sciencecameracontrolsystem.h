#ifndef SCIENCECAMERACONTROLSYSTEM_H
#define SCIENCECAMERACONTROLSYSTEM_H

#include <QObject>
#include <QTimer>

#include "settingsmodel.h"

#include <qmqtt/qmqtt.h>
#include <SDL2/SDL.h>

namespace Soro {

/* Controls the scientific camera gimbal's movement from input
 * on a gamepad
 */
class ScienceCameraControlSystem: public QObject
{
    Q_OBJECT
public:
    explicit ScienceCameraControlSystem(const SettingsModel *settings, QObject *parent=0);

    SettingsModel::CameraGimbalInputMode getInputMode() const;
    void setInputMode(SettingsModel::CameraGimbalInputMode mode);

Q_SIGNALS:
    void sciencePackageControllerDisconnected();
    void sciencePackageDisconnected();
    void flirDisconnected();
    void lidarDisconnected();

public Q_SLOTS:
    void enable();
    void disable();

    void onGamepadAxisUpdate(SDL_GameControllerAxis axis, float value);

private:
    QMQTT::Client *_mqtt;
    QTimer _timer;
    quint16 _nextMqttMsgId;
    SettingsModel::CameraGimbalInputMode _mode;
    float _gamepadLeftX, _gamepadLeftY, _gamepadRightX, _gamepadRightY;
    bool _enabled;

};

} // namespace Soro

#endif // SCIENCECAMERACONTROLSYSTEM_H
