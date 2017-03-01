#include "drivecontrolsystem.h"


#define LOG_TAG "DriveControlSystem"

namespace Soro{

DriveControlSystem::DriveControlSystem(QObject *parent) : QObject(parent){

    sendTimerId = startTimer(1000);
}

void DriveControl::setGamepad(GamepadController *gamepad)
{
    _gamepad = gamepad;
    _gamepadName = gamepad->getGamepadName();
}

 Soro::Messages::drive_ buildDriveMessage(){











}

 //Timer loop to get latest controller axis changes
void timerEvent(QTimerEvent* e){

     if(e->timerId() == sendTimerId){

         MainController::getGamepadController();



         //Soro::Messages::drive_ *driveMessage = buildDriveMessage(axis, value);

         ros::NodeHandle driveHandle;

         ros::Publisher drivePublisher = driveHandle.advertise<Soro::Messages::drive_>("drive");

         drivePublisher.publish(driveMessage);
     }

 }


}
