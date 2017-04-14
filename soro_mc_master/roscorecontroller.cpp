#include "roscorecontroller.h"
#include "maincontroller.h"
#include "libsoromc/constants.h"
#include "libsoromc/logger.h"

#include  <signal.h>
#include  <sys/ipc.h>

#define LogTag "RoscoreController"

namespace Soro {

RosCoreController::RosCoreController(QObject *parent) : QObject(parent)
{
    Logger::logInfo(LogTag, QString("Starting bash host process with roscore on port %1").arg(SORO_MC_ROS_MASTER_PORT));
    connect(&_roscoreProcess, &QProcess::stateChanged, this, &RosCoreController::roscoreProcessStateChanged);
    _roscoreProcess.start("/bin/bash", QStringList() << "-c" << QString("source /opt/ros/kinetic/setup.bash; exec roscore -p %1").arg(SORO_MC_ROS_MASTER_PORT));
}

RosCoreController::~RosCoreController()
{
    disconnect(&_roscoreProcess, &QProcess::stateChanged, this, &RosCoreController::roscoreProcessStateChanged);
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
        MainController::panic("Roscore has terminated unexpectedly."
                              "This may be because another master is already running, or ROS isn't installed or configured correctly.");
        break;
    }
}

} // namespace Soro

