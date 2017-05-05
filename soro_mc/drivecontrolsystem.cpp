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
#include "soro_core/logger.h"
#include <SDL2/SDL.h>
#include <climits>

#define LogTag "DriveControlSystem"

namespace Soro
{

DriveControlSystem::DriveControlSystem(int sendInterval, const GamepadController *gamepad,
                                       ConnectionStatusController *connectionStatusController,
                                       QObject *parent) : QObject(parent)
{
    _gamepad = gamepad;
    _connectionStatusController = connectionStatusController;
    setSkidSteerFactor(0.2);

    _sendTimerId = startTimer(qMax<int>(sendInterval, 10));
    Logger::logInfo(LogTag, "Creating publisher...");
    _drivePublisher = _nh.advertise<ros_generated::drive>("drive", 1);
    Logger::logInfo(LogTag, "Publisher created");
}

void DriveControlSystem::setSkidSteerFactor(float factor)
{
    if (factor > 1.0) _skidSteerFactor = 1.0;
    else if (factor < 0.0) _skidSteerFactor = 0.0;
    else _skidSteerFactor = factor;
}

float DriveControlSystem::getSkidSteerFactor() const
{
    return _skidSteerFactor;
}

void DriveControlSystem::setLimit(float limit)
{
    if (limit > 1.0) _limit = 1.0;
    else if (limit < 0.0) _limit = 0.0;
    else _limit = limit;
}

float DriveControlSystem::getLimit() const
{
    return _limit;
}

ros_generated::drive DriveControlSystem::buildDriveMessage()
{
    qint8 leftY = _gamepad->getAxisValue(SDL_CONTROLLER_AXIS_LEFTY) * -1 * (INT8_MAX - 1) * _limit;
    qint8 rightY = _gamepad->getAxisValue(SDL_CONTROLLER_AXIS_RIGHTY) * -1 * (INT8_MAX - 1) * _limit;
    float midScale = _skidSteerFactor * ((float)qAbs(leftY - rightY) / 128.0);

    ros_generated::drive driveMessage;
    driveMessage.wheelBL = leftY;
    driveMessage.wheelML = leftY - (qint8)(midScale * leftY);
    driveMessage.wheelFL = leftY;
    driveMessage.wheelBR = rightY;
    driveMessage.wheelMR = rightY - (qint8)(midScale * rightY);
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
        _connectionStatusController->logDataUp(6);
    }
}

} // namespace Soro
