#ifndef DRIVECONTROLSYSTEM_H
#define DRIVECONTROLSYSTEM_H

#include <QObject>
#include <gamepadcontroller.h>
#include "libsoromc/drivemessage.h"
#include "ros/ros.h"
#include <SDL2/SDL.h>
#include <QTimerEvent>

namespace Soro {

class DriveControlSystem : public QObject
{
    Q_OBJECT

public:
    explicit DriveControlSystem(QObject *parent = 0);

    void setGamepad(GamepadController *gamepad);

    void buildDriveMessage(SDL_GameControllerAxis *axis, float value);

private:
    GamepadController *_gamepad = nullptr;
    QString _gamepadName;

    int sendTimerId;

    int8_t wheelIFL;
    int8_t wheelIFR;
    int8_t wheelIML;
    int8_t wheelIMR;
    int8_t wheelIBL;
    int8_t wheelIBR;

public slots:
    void driveControlSystemSlot();

protected:
    void timerEvent(QTimerEvent* e);

};


}

#endif // DRIVECONTROLSYSTEM_H
