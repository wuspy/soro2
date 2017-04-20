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

#include "armcontrolsystem.h"
#include "maincontroller.h"
#include "libsoromc/logger.h"

#define LogTag "ArmControlSystem"

namespace Soro
{

ArmControlSystem::ArmControlSystem(QObject *parent) : QObject(parent)
{
    Logger::logInfo(LogTag, "Creating publisher...");
    _armPublisher = _nh.advertise<std_msgs::UInt8MultiArray>("arm", 1);
    if (!_armPublisher) MainController::panic(LogTag, "Failed to create ROS publisher for arm topic");
    Logger::logInfo(LogTag, "Arm Publisher created");
}

}
