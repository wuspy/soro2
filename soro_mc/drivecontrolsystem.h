#ifndef DRIVECONTROLSYSTEM_H
#define DRIVECONTROLSYSTEM_H

#include <QObject>
#include <gamepadcontroller.h>

namespace Soro {

class DriveControlSystem : public QObject
{
    Q_OBJECT

public:
    explicit DriveControlSystem(QObject *parent = 0);

    void setGamepad(GamepadController *gamepad);

private:
    GamepadController *_gamepad = nullptr;
    QString _gamepadName;

    int8_t wheelIFL;
    int8_t wheelIFR;
    int8_t wheelIML;
    int8_t wheelIMR;
    int8_t wheelIBL;
    int8_t wheelIBR;

public slots:
    void setAxis(SDL_GameControllerAxis axis, float value);

};


}

#endif // DRIVECONTROLSYSTEM_H
