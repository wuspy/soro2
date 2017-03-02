#include "drivecontrolsystem.h"
#include "maincontroller.h"
#include <SDL2/SDL.h>
#include <climits>

#define LOG_TAG "DriveControlSystem"

// ***TODO***
// ROS hangs the application if connection to master is not immediately successful
// All functionality associated with ROS has been commented out until this can be resolved
namespace Soro
{

DriveControlSystem::DriveControlSystem(QObject *parent) : QObject(parent){
    // TODO allow adjustment of send interval
    sendTimerId = startTimer(20);
    //drivePublisher = driveHandle.advertise<Soro::Messages::drive>("drive", 10);
}

Soro::Messages::drive DriveControlSystem::buildDriveMessage()
{
    // TODO support different input modes
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
