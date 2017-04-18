#include "armcontrolsystem.h"
#include "maincontroller.h"

#define LogTag "ArmControlSystem"

namespace Soro
{

ArmControlSystem::ArmControlSystem(QObject *parent) : QObject(parent)
{
    MainController::logInfo(LogTag, "Creating publisher...");
    _armPublisher = MainController::getNodeHandle()->advertise<std_msgs::UInt8MultiArray>("arm", 1);
    MainController::logInfo(LogTag, "Arm Publisher created");
}

}
