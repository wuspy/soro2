#ifndef MAINWINDOWCONTROLLER_H
#define MAINWINDOWCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QQuickWindow>

#include <SDL2/SDL.h>

namespace Soro {

class MainWindowController : public QObject
{
    Q_OBJECT
public:
    explicit MainWindowController(QQmlEngine *engine, QObject *parent = 0);

private:
    QQuickWindow *_window;

private slots:
    void onGamepadButtonPressed(SDL_GameControllerButton button, bool pressed);
};

} // namespace Soro

#endif // MAINWINDOWCONTROLLER_H
