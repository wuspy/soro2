/*
 * Copyright 2017 The University of Oklahoma.
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
    _drivePublisher = MainController::getNodeHandle()->advertise<ros_generated::drive>("drive", 1);
    Logger::logInfo(LogTag, "Publisher created");
}

ros_generated::drive DriveControlSystem::buildDriveMessage()
{
    // TODO support different input modes
    qint8 leftY = MainController::getGamepadController()->getAxisValue(SDL_CONTROLLER_AXIS_LEFTY) * -0.5 * (INT8_MAX - 1);
    qint8 rightY = MainController::getGamepadController()->getAxisValue(SDL_CONTROLLER_AXIS_RIGHTY) * -0.5 * (INT8_MAX - 1);
    ros_generated::drive driveMessage;
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
        ros_generated::drive driveMessage = buildDriveMessage();
        _drivePublisher.publish(driveMessage);
        MainController::getConnectionStatusController()->logBitsUp(6 * 8);
    }
}

} // namespace Soro
