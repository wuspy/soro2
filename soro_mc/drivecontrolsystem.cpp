#include "drivecontrolsystem.h"
#include "maincontroller.h"
#include "libsoromc/logger.h"
#include <SDL2/SDL.h>
#include <climits>

#define LogTag "DriveControlSystem"

namespace Soro
{

DriveControlSystem::DriveControlSystem(QObject *parent) : QObject(parent)
{
    // TODO allow adjustment of send interval
    _sendTimerId = startTimer(20);
    Logger::logInfo(LogTag, "Creating publisher...");
    _drivePublisher = MainController::getNodeHandle()->advertise<Soro::Messages::drive>("drive", 1);
    Logger::logInfo(LogTag, "Publisher created");
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
    if(e->timerId() == _sendTimerId)
    {
        Soro::Messages::drive driveMessage = buildDriveMessage();
        _drivePublisher.publish(driveMessage);
        MainController::getConnectionStatusController()->logBitsUp(6 * 8);
    }
}

} // namespace Soro
