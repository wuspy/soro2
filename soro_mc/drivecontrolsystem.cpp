#include "drivecontrolsystem.h"
#include "maincontroller.h"

#define LOG_TAG "DriveControlSystem"

namespace Soro{

DriveControlSystem::DriveControlSystem(QObject *parent) : QObject(parent){

    _drivePublisher = _driveHandle.advertise<Soro::Messages::drive>("drive", 1000);

    _sendTimerId = startTimer(1000);
}


Soro::Messages::drive* DriveControlSystem::buildDriveMessage(float x, float y){

    Soro::Messages::drive *message = new Soro::Messages::drive();

    if(_mode == 1){

        message->wheelML = x * 100;

        message->wheelMR = y * 100;

    }
    else{

        message->wheelFL = x;
        message->wheelML = x * 100;
        message->wheelBL = x;

        message->wheelFR = y;
        message->wheelMR = y * 100;
        message->wheelBR = y;
    }

    return message;

}

 //Timer loop to get latest controller axis changes
void DriveControlSystem::timerEvent(QTimerEvent* e){

    float x;
    float y;

     if(e->timerId() == _sendTimerId){

        //In drive control mode
         if(_enabled){

             Soro::Messages::drive* driveMessage;

             //single stick mode
             if(_mode == 1){

                 x = MainController::getGamepadController()->getAxisValue(SDL_CONTROLLER_AXIS_LEFTX);

                 y = MainController::getGamepadController()->getAxisValue(SDL_CONTROLLER_AXIS_LEFTY);

                 driveMessage = buildDriveMessage(x, y);


             //dual stick mode
             } else{

                 x = MainController::getGamepadController()->getAxisValue(SDL_CONTROLLER_AXIS_LEFTY);

                 y = MainController::getGamepadController()->getAxisValue(SDL_CONTROLLER_AXIS_RIGHTY);

                 driveMessage = buildDriveMessage(x, y);

             }


             _drivePublisher.publish(*driveMessage);

         }
     }

 }

void DriveControlSystem::setDriveMode(int i){

    if(i == 1){
        MainController::logInfo(LOG_TAG, "Setting drive mode to SINGLE");
    }
    else{
        MainController::logInfo(LOG_TAG, "Setting drive mode to DUAL");
    }

    _mode = i;
}

int DriveControlSystem::getDriveMode(){
    return _mode;
}

void DriveControlSystem::setDriveEnabled(bool e){

    if(e){
        MainController::logInfo(LOG_TAG, "Enabling Drive Control");
    }
    else{
        MainController::logInfo(LOG_TAG, "Disabling Drive Control");
    }
    _enabled = e;
}

bool DriveControlSystem::getDriveEnabled(){
    return _enabled;
}


}
