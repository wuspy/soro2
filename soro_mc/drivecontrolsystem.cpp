#include "drivecontrolsystem.h"

#define LOG_TAG "DriveControlSystem"

namespace Soro{

DriveControlSystem::DriveControlSystem(QObject *parent) : QObject(parent){}

void DriveControl::setGamepad(GamepadController *gamepad)
{
    _gamepad = gamepad;
    _gamepadName = gamepad->getGamepadName();
}

void setAxis(SDL_GameControllerAxis axis, float value){

    switch(axis)
    {
    case SDL_CONTROLLER_AXIS_LEFTX

    }
}

}
