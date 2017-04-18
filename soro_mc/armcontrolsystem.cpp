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
