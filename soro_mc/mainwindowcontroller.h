#ifndef MAINWINDOWCONTROLLER_H
#define MAINWINDOWCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QQuickWindow>

namespace Soro {

class MainWindowController : public QObject
{
    Q_OBJECT
public:
    explicit MainWindowController(QQmlEngine *engine, QObject *parent = 0);

private:
    QQuickWindow *_window;
};

} // namespace Soro

#endif // MAINWINDOWCONTROLLER_H
