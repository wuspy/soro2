/*
 * Copyright 2016 The University of Oklahoma.
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

#include "gamepadcontroller.h"
#include <climits>

#define LOG_TAG "GamepadController"

namespace Soro {

GamepadController::GamepadController(QObject *parent) : QObject(parent) { }

float convertToFloat(qint16 value) \
{
    return (float) value/INT16_MAX;
}

SDL_GameController* GamepadController::getGamepad()
{
    return _gameController;
}

QString GamepadController::getGamepadName() const
{
    return _gamepadName;
}

bool GamepadController::isGamepadConnected() const {
    return _gameController != nullptr;
}

bool GamepadController::getButtonPressed(SDL_GameControllerButton button)
{
    return SDL_GameControllerGetButton(_gameController, button);
}

float GamepadController::getAxisValue(SDL_GameControllerAxis axis)
{
    return SDL_GameControllerGetAxis(_gameController, axis);
}

void GamepadController::setTimerId(int id)
{
    _timerId = id;
}

int GamepadController::getTimerId() const
{
    return _timerId;
}

void GamepadController::setPollInterval(int rate)
{
    _pollInterval = rate;
    int timer_id = getTimerId();
//    LOG_I(LOG_TAG, QString("The %1's polling rate has been changed").arg(_gamepadName));
    if (timer_id) {
        killTimer(timer_id);
        setTimerId(startTimer(_pollInterval));
    }
}

int GamepadController::getPollInterval() const
{
    return _pollInterval;
}

void GamepadController::timerEvent(QTimerEvent *e)
{
    QObject::timerEvent(e);
    if (e->timerId() == _timerId)
    {
        SDL_GameControllerUpdate();
        if (_gameController && SDL_GameControllerGetAttached(_gameController))
        {
            // Update gamepad data
            _axisLeftX           = SDL_GameControllerGetAxis(_gameController, SDL_CONTROLLER_AXIS_LEFTX);
            _axisLeftY           = SDL_GameControllerGetAxis(_gameController, SDL_CONTROLLER_AXIS_LEFTY);
            _axisRightX          = SDL_GameControllerGetAxis(_gameController, SDL_CONTROLLER_AXIS_RIGHTX);
            _axisRightY          = SDL_GameControllerGetAxis(_gameController, SDL_CONTROLLER_AXIS_RIGHTY);
            _axisLeftTrigger     = SDL_GameControllerGetAxis(_gameController, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
            _axisRightTrigger    = SDL_GameControllerGetAxis(_gameController, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
            _buttonA             = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_A);
            _buttonB             = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_B);
            _buttonX             = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_X);
            _buttonY             = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_Y);
            _buttonLeftShoulder  = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
            _buttonRightShoulder = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
            _buttonStart         = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_START);
            _buttonBack          = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_BACK);
            _buttonLeftStick     = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_LEFTSTICK);
            _buttonRightStick    = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_RIGHTSTICK);
            _dpadUp              = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_DPAD_UP);
            _dpadLeft            = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
            _dpadDown            = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
            _dpadRight           = SDL_GameControllerGetButton(_gameController, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
            emit poll();
        }
        else
        {
            for (int i = 0; i < SDL_NumJoysticks(); ++i)
            {
                if (SDL_IsGameController(i))
                {
                    SDL_GameController *controller = SDL_GameControllerOpen(i);
                    if (controller) {
                        //this gamepad will do
                        setGamepad(controller);
                        setTimerId(startTimer(getPollInterval()));
                        break;
                    }
                    SDL_GameControllerClose(controller);
                    //            LOG_I(LOG_TAG, "The gamepad has been disconnected");
                }
            }
        }
    }
}

void GamepadController::setGamepad(SDL_GameController *controller)
{
    if (_gameController != controller)
    {
        _gameController = controller;
        _gamepadName = _gameController ? QString(SDL_GameControllerName(_gameController)) : "";
//        LOG_I(LOG_TAG, "Active controller is \'" + _gamepadName + "\'");
        emit gamepadChanged(isGamepadConnected(), _gamepadName);
    }
}

}
