#ifndef GAMEPADCONTROLLER_H
#define GAMEPADCONTROLLER_H

#include <QObject>
#include <SDL2/SDL.h>

namespace Soro {

class GamepadController : public QObject
{
    Q_OBJECT
public:
    explicit GamepadController(QObject *parent = 0);

    void setPollInterval(int rate);

    int getPollInterval() const;

    /* Gets the name of the currently connected gamepad, or an empty string
     * if no gamepad is connected.
     */
    QString getGamepadName() const;

    /* Returns true if a gamepad is current connected and being polled
     */
    bool isGamepadConnected() const;

    bool getButtonPressed(SDL_GameControllerButton button);
    float getAxisValue(SDL_GameControllerAxis axis);

signals:
    void buttonPressed(SDL_GameControllerButton button, bool isPressed);
    void axisChanged(SDL_GameControllerAxis axis, float value);
    void gamepadChanged(bool isConnected, QString name);

private:
    int _pollInterval = 10;
};

} // namespace Soro

#endif // GAMEPADCONTROLLER_H
