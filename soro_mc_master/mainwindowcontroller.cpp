#include "mainwindowcontroller.h"

#include <QQmlComponent>

MainWindowController::MainWindowController(QQmlEngine *engine, QObject *parent) : QObject(parent)
{
    QQmlComponent qmlComponent(engine, QUrl("qrc:/main.qml"));
    _window = qobject_cast<QQuickWindow*>(qmlComponent.create());
    if (!qmlComponent.errorString().isEmpty() || !_window)
    {
        // There was an error creating the QML window. This is most likely due to a QML syntax error
        throw QString("Failed to create QML window:  %1").arg(qmlComponent.errorString());
    }
}

void MainWindowController::onConnectedChanged(bool connected)
{
    _window->setProperty("connected", connected);
}

void MainWindowController::onLatencyUpdated(quint32 latency)
{
    _window->setProperty("latency", latency);
}

void MainWindowController::onBitrateUpdated(quint64 bitsUp, quint64 bitsDown)
{
    _window->setProperty("bitrateUp", bitsUp);
    _window->setProperty("bitrateDown", bitsDown);
}
