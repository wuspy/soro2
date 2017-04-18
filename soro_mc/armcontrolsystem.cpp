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
    Logger::logInfo(LogTag, "Arm Publisher created");
}

}
