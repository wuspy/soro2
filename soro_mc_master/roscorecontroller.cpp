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

#include "roscorecontroller.h"
#include "maincontroller.h"
#include "libsoromc/constants.h"
#include "libsoromc/logger.h"

#define LogTag "RosCoreController"

namespace Soro {

RosCoreController::RosCoreController(QObject *parent) : QObject(parent)
{
    Logger::logInfo(LogTag, QString("Starting bash host process with roscore on port %1").arg(SORO_MC_ROS_MASTER_PORT));
    connect(&_roscoreProcess, &QProcess::stateChanged, this, &RosCoreController::roscoreProcessStateChanged);

    // Start a bash process that will setup the environment ROS needs and exec to roscore
    _roscoreProcess.start("/bin/bash", QStringList() << "-c" << QString("source /opt/ros/kinetic/setup.bash; exec roscore -p %1").arg(SORO_MC_ROS_MASTER_PORT));
}

RosCoreController::~RosCoreController()
{
    disconnect(&_roscoreProcess, &QProcess::stateChanged, this, &RosCoreController::roscoreProcessStateChanged);

    // Stop roscore
    if (_roscoreProcess.state() != QProcess::NotRunning)
    {
        Logger::logInfo(LogTag, "Sending SIGTERM to roscore...");
        _roscoreProcess.terminate();
        if (!_roscoreProcess.waitForFinished(5000))
        {
            Logger::logError(LogTag, "roscore has not exited after 5 seconds, sending SIGKILL");
            _roscoreProcess.kill();
            _roscoreProcess.waitForFinished();
        }

        Logger::logInfo(LogTag, "roscore has exited");
    }
}

void RosCoreController::roscoreProcessStateChanged(QProcess::ProcessState state)
{
    switch (state)
    {
    case QProcess::Starting:
        Logger::logInfo(LogTag, "roscore is now in starting state");
        break;
    case QProcess::Running:
        Logger::logInfo(LogTag, "roscore is now in running state");
        break;
    case QProcess::NotRunning:
        MainController::panic(LogTag, "Roscore has terminated unexpectedly."
                              "This may be because another master is already running, or ROS isn't installed or configured correctly.");
        break;
    }
}

} // namespace Soro

