/*
 * Copyright 2017 The University of Oklahoma.
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

#include "mainwindowcontroller.h"
#include "maincontroller.h"
#include "qmlgstreamerglitem.h"
#include "qmlgstreamerpainteditem.h"
#include "soro_core/constants.h"
#include "soro_core/logger.h"
#include "soro_core/gstreamerutil.h"

#include <QQmlComponent>

#include <Qt5GStreamer/QGst/ElementFactory>

#define LogTag "MainWindowController"

namespace Soro {

MainWindowController::MainWindowController(QQmlEngine *engine, const SettingsModel *settings,
                                           const CameraSettingsModel *cameraSettings, QObject *parent) : QObject(parent)
{
    _cameraSettings = cameraSettings;
    _settings = settings;

    QQmlComponent qmlComponent(engine, QUrl("qrc:/qml/main.qml"));
    _window = qobject_cast<QQuickWindow*>(qmlComponent.create());
    if (!qmlComponent.errorString().isEmpty() || !_window)
    {
        // There was an error creating the QML window. This is most likely due to a QML syntax error
        MainController::panic(LogTag, QString("Failed to create QML window:  %1").arg(qmlComponent.errorString()));
    }
    //
    // Setup the camera views in the UI
    //
    int videoCount = cameraSettings->getCameraCount();
    if (videoCount > 10)
    {
        MainController::panic(LogTag, "UI does not support more than 10 different camera views");
    }
    _window->setProperty("videoCount", videoCount);

    for (int i = 0; i < videoCount; i++)
    {
        // Set camera name
        _window->setProperty(QString("video%1Name").arg(i).toLatin1().constData(), cameraSettings->getCamera(i).name);
    }

    _window->setProperty("selectedView", "video0");

    connect(_window, SIGNAL(keyPressed(int)), this, SIGNAL(keyPressed(int)));

    //
    // Setup ROS communication
    //

    Logger::logInfo(LogTag, "Creating ROS publisher and subscriber for notificaiton topic...");
    _notifyPublisher = _nh.advertise<ros_generated::notification>("notification", 10);
    _notifySubscriber = _nh.subscribe
            <ros_generated::notification, Soro::MainWindowController>
            ("notification", 10, &MainWindowController::onNewNotification, this);
    Logger::logInfo(LogTag, "ROS publisher and subscriber created");
}

QVector<QGst::ElementPtr> MainWindowController::getVideoSinks()
{
    QVector<QGst::ElementPtr> sinks;
    for (int i = 0; i < 10; ++i)
    {
        if (_settings->getEnableHwRendering())
        {
            // Item should be of the class QmlGStreamerGlItem
            sinks.append(qvariant_cast<QmlGStreamerGlItem*>(_window->property(QString("video%1Surface").arg(i).toLatin1().constData()))->videoSink());
        }
        else
        {
            // Item should be of the class QmlGStreamerPaintedItem
            sinks.append(qvariant_cast<QmlGStreamerPaintedItem*>(_window->property(QString("video%1Surface").arg(i).toLatin1().constData()))->videoSink());
        }
    }
    return sinks;
}

void MainWindowController::onNewNotification(ros_generated::notification msg)
{
    // Show this message in the UI
    QString title = QString(msg.title.c_str());
    QString message = QString(msg.message.c_str());
    notify((int)msg.type, title, message);

    switch (msg.type)
    {
    case NOTIFICATION_TYPE_ERROR:
        Logger::logError(LogTag, QString("Received error notification: %1 - %2 ").arg(title, message));
        break;
    case NOTIFICATION_TYPE_WARNING:
        Logger::logWarn(LogTag, QString("Received warning notification: %1 - %2 ").arg(title, message));
        break;
    case NOTIFICATION_TYPE_INFO:
        Logger::logInfo(LogTag, QString("Received info notification: %1 - %2 ").arg(title, message));
        break;
    }
}

void MainWindowController::notify(int type, QString title, QString message)
{
    QString typeString;
    switch (type)
    {
    case NOTIFICATION_TYPE_INFO:
        typeString = "info";
        break;
    case NOTIFICATION_TYPE_WARNING:
        typeString = "warning";
        break;
    case NOTIFICATION_TYPE_ERROR:
        typeString = "error";
        break;
    }

    QMetaObject::invokeMethod(_window, "notify", Q_ARG(QVariant, typeString), Q_ARG(QVariant, title), Q_ARG(QVariant, message));
}

void MainWindowController::notifyAll(int type, QString title, QString message)
{
    ros_generated::notification msg;
    msg.type = (uint8_t)type;
    msg.title = title.toStdString();
    msg.message = message.toStdString();

    // Publish this notification on the notification topic. We will get this message back,
    // since we are also subscribed to it, and that's when we'll show it from the onNewNotification() function
    _notifyPublisher.publish(msg);
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

void MainWindowController::onGamepadButtonPressed(SDL_GameControllerButton button, bool pressed)
{
    if (pressed)
    {
        switch (button)
        {
        case SDL_CONTROLLER_BUTTON_Y:
            //
            // Toggle sidebar
            //
            QMetaObject::invokeMethod(_window, "toggleSidebar");
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_UP:
            //
            // Move to view above
            //
            QMetaObject::invokeMethod(_window, "selectViewAbove");
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
            //
            // Move to view below
            //
            QMetaObject::invokeMethod(_window, "selectViewBelow");
            break;
        default:
            break;
        }
    }
}

} // namespace Soro
