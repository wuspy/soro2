#ifndef GAMEPADCONTROLLER_H
#define GAMEPADCONTROLLER_H

#include <QObject>
#include <QTimerEvent>
#include <SDL2/SDL.h>

namespace Soro {

class GamepadController : public QObject
{
    Q_OBJECT
public:
    explicit GamepadController(QObject *parent = 0);

    /* Sets the poll rate for how often the code checks for a new controller
     * to be connected or if a button has been presed and restarts the timer.
     * Gets the pool rate.
    */
    void setPollInterval(int rate);
    int getPollInterval() const;

    float getDeadzone() const;
    void setDeadzone(float deadzone);

    /* Gets the currently connected gamepad, or null if no gamepad is connected.
     */
    SDL_GameController* getGamepad();

    /* Gets the name of the currently connected gamepad, or an empty string
     * if no gamepad is connected.
     */
    QString getGamepadName() const;

    /* Returns true if a gamepad is current connected and being polled
     */
    bool isGamepadConnected() const;

    /* Returns true if button is pressed or value of axis respectively  */
    bool getButtonPressed(SDL_GameControllerButton button);
    float getAxisValue(SDL_GameControllerAxis axis);

    void updateIfChangedAxis(SDL_GameControllerAxis axis, qint16 *currentValue);
    void updateIfChangedButton(SDL_GameControllerButton button, bool *currentValue);

signals:
    /* Emitted when a gamepad button is pressed */
    void buttonPressed(SDL_GameControllerButton button, bool isPressed);
    /* Emitted when a gamepad joystick is moved */
    void axisChanged(SDL_GameControllerAxis axis, float value);
    /* Emitted when a gamepad is connected/removed */
    void gamepadChanged(bool isConnected, QString name);
    /* Emitted when new values are read from the gamepad */
    void poll();

protected:
    void timerEvent(QTimerEvent *event);

private:
    int _pollInterval;
    int _timerId;
    float _deadzone;

    /* Sets the currently active controller to be polled */
    void setGamepad(SDL_GameController *controller);
    float convertToFloatWithDeadzone(qint16 value, float deadzone);

    SDL_GameController *_gameController = nullptr;
    QString _gamepadName;

    /* Axises for controller initialized to dead center */
    qint16 _axisLeftX = 0, _axisLeftY = 0, _axisRightX = 0, _axisRightY = 0,
            _axisLeftTrigger = 0, _axisRightTrigger = 0;

    /* Buttons for controller initialized to not pressed  */
    bool _buttonA = false, _buttonB = false, _buttonX = false, _buttonY = false,
        _buttonLeftShoulder = false, _buttonRightShoulder = false,
        _buttonLeftStick = false, _buttonRightStick = false,
        _buttonBack = false, _buttonStart = false, _dpadUp = false,
        _dpadLeft = false, _dpadRight = false, _dpadDown = false;
};

} // namespace Soro

#endif // GAMEPADCONTROLLER_H
