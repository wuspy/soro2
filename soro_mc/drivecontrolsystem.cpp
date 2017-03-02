#include "drivecontrolsystem.h"
#include "maincontroller.h"
#include <SDL2/SDL.h>
#include <climits>

#define LOG_TAG "DriveControlSystem"

namespace Soro
{

DriveControlSystem::DriveControlSystem(QObject *parent) : QObject(parent){
    //sendTimerId = startTimer(20);
    //drivePublisher = driveHandle.advertise<Soro::Messages::drive>("drive", 10);
}

Soro::Messages::drive DriveControlSystem::buildDriveMessage()
{
    qint8 leftY = MainController::getGamepadController()->getAxisValue(SDL_CONTROLLER_AXIS_LEFTY) * -0.5 * (INT8_MAX - 1);
    qint8 rightY = MainController::getGamepadController()->getAxisValue(SDL_CONTROLLER_AXIS_RIGHTY) * -0.5 * (INT8_MAX - 1);
    Soro::Messages::drive driveMessage;
    driveMessage.wheelBL = leftY;
    driveMessage.wheelML = leftY;
    driveMessage.wheelFL = leftY;
    driveMessage.wheelBR = rightY;
    driveMessage.wheelMR = rightY;
    driveMessage.wheelFR = rightY;

    return driveMessage;
}

//Timer loop to get latest controller axis changes
void DriveControlSystem::timerEvent(QTimerEvent* e)
{
    if(e->timerId() == sendTimerId)
    {
        Soro::Messages::drive driveMessage = buildDriveMessage();

        //drivePublisher.publish(driveMessage);
    }
}

} // namespace Soro
