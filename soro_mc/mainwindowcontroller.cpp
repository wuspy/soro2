/*
 * Copyright 2017 Jacob Jordan <doublejinitials@ou.edu>
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

MainWindowController::MainWindowController(QQmlEngine *engine, const SettingsModel *settings, const MediaProfileSettingsModel *mediaProfileSettings,
                                           const CameraSettingsModel *cameraSettings, QObject *parent) : QObject(parent)
{
    _cameraSettings = cameraSettings;
    _mediaProfileSettings = mediaProfileSettings;
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
        //_window->setProperty(QString("video%1Name").arg(i).toLatin1().constData(), cameraSettings->getCamera(i).name);
        QMetaObject::invokeMethod(_window, "setVideoName", Q_ARG(QVariant, i), Q_ARG(QVariant, cameraSettings->getCamera(i).name));
    }

    _window->setProperty("selectedView", "video0");

    switch (settings->getConfiguration())
    {
    case SettingsModel::ArmOperatorConfiguration:
        _window->setProperty("configuration", "Arm Operator");
        break;
    case SettingsModel::DriverConfiguration:
        _window->setProperty("configuration", "Driver");
        break;
    case SettingsModel::ScienceArmOperatorConfiguration:
        _window->setProperty("configuration", "Science Arm Operator");
        break;
    case SettingsModel::ScienceCameraOperatorConfiguration:
        _window->setProperty("configuration", "Science Camera Operator");
        break;
    case SettingsModel::CameraOperatorConfiguration:
        _window->setProperty("configuration", "Camera Operator");
        break;
    }

    connect(_window, SIGNAL(keyPressed(int)), this, SIGNAL(keyPressed(int)));

    //
    // Setup MQTT
    //
    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    connect(_mqtt, &QMQTT::Client::received, this, &MainWindowController::onMqttMessage);
    connect(_mqtt, &QMQTT::Client::connected, this, &MainWindowController::onMqttConnected);
    connect(_mqtt, &QMQTT::Client::disconnected, this, &MainWindowController::onMqttDisconnected);
    _mqtt->setClientId(MainController::getId() + "_mainwindowcontroller");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->connectToHost();
}

void MainWindowController::onMqttConnected()
{
    LOG_I(LogTag, "Connected to MQTT broker");
    _mqtt->subscribe("notification", 2);
    Q_EMIT mqttConnected();
}

void MainWindowController::onMqttDisconnected()
{
    LOG_W(LogTag, "Disconnected from MQTT broker");
    Q_EMIT mqttDisconnected();
}

void MainWindowController::onMqttMessage(const QMQTT::Message &msg)
{
    if (msg.topic() == "notification")
    {
        NotificationMessage notificationMsg(msg.payload());

        switch (notificationMsg.level)
        {
        case NotificationMessage::Level_Error:
            LOG_E(LogTag, QString("Received error notification: %1 - %2 ").arg(notificationMsg.title, notificationMsg.message));
            break;
        case NotificationMessage::Level_Warning:
            LOG_W(LogTag, QString("Received warning notification: %1 - %2 ").arg(notificationMsg.title, notificationMsg.message));
            break;
        case NotificationMessage::Level_Info:
            LOG_I(LogTag, QString("Received info notification: %1 - %2 ").arg(notificationMsg.title, notificationMsg.message));
            break;
        }
    }
}

QVector<QGst::ElementPtr> MainWindowController::getVideoSinks()
{
    QVector<QGst::ElementPtr> sinks;
    for (int i = 0; i < _cameraSettings->getCameraCount(); ++i)
    {
        if (_settings->getEnableHwRendering())
        {
            // Item should be of the class QmlGStreamerGlItem
            QVariant returnValue;
            QMetaObject::invokeMethod(_window, "getVideoSurface", Q_RETURN_ARG(QVariant, returnValue), Q_ARG(QVariant, i));
            //sinks.append(qvariant_cast<QmlGStreamerGlItem*>(_window->property(QString("video%1Surface").arg(i).toLatin1().constData()))->videoSink());
            sinks.append(qvariant_cast<QmlGStreamerGlItem*>(returnValue)->videoSink());
        }
        else
        {
            // Item should be of the class QmlGStreamerPaintedItem
            QVariant returnValue;
            QMetaObject::invokeMethod(_window, "getVideoSurface", Q_RETURN_ARG(QVariant, returnValue), Q_ARG(QVariant, i));
            //sinks.append(qvariant_cast<QmlGStreamerPaintedItem*>(_window->property(QString("video%1Surface").arg(i).toLatin1().constData()))->videoSink());
            sinks.append(qvariant_cast<QmlGStreamerPaintedItem*>(returnValue)->videoSink());
        }
    }
    return sinks;
}

void MainWindowController::notify(NotificationMessage::Level level, QString title, QString message)
{
    QString typeString;
    switch (level)
    {
    case NotificationMessage::Level_Error:
        typeString = "error";
        break;
    case NotificationMessage::Level_Warning:
        typeString = "warning";
        break;
    case NotificationMessage::Level_Info:
        typeString = "info";
        break;
    }

    QMetaObject::invokeMethod(_window, "notify", Q_ARG(QVariant, typeString), Q_ARG(QVariant, title), Q_ARG(QVariant, message));
}

void MainWindowController::notifyAll(NotificationMessage::Level level, QString title, QString message)
{
    NotificationMessage msg;
    msg.level = level;
    msg.title = title;
    msg.message = message;

    // Publish this notification on the notification topic. We will get this message back,
    // since we are also subscribed to it, and that's when we'll show it from the onNewNotification() function
    _mqtt->publish(QMQTT::Message(_notificationMsgId++, "notification", msg, 2));
}

void MainWindowController::onConnectedChanged(bool connected)
{
    _window->setProperty("connected", connected);
}

void MainWindowController::onLatencyUpdated(quint32 latency)
{
    _window->setProperty("latency", latency);
}

void MainWindowController::onDataRateUpdated(quint64 rateFromRover)
{
    _window->setProperty("dataRateFromRover", rateFromRover);
}

void MainWindowController::toggleSidebar()
{
    QMetaObject::invokeMethod(_window, "toggleSidebar");
}

void MainWindowController::selectViewAbove()
{
    QMetaObject::invokeMethod(_window, "selectViewAbove");
}

void MainWindowController::selectViewBelow()
{
    QMetaObject::invokeMethod(_window, "selectViewBelow");
}

void MainWindowController::dismissNotification()
{
    QMetaObject::invokeMethod(_window, "dismissNotification");
}

void MainWindowController::onAudioProfileChanged(GStreamerUtil::AudioProfile profile)
{
    _window->setProperty("audioStreaming", profile.codec != GStreamerUtil::CODEC_NULL);
    _window->setProperty("audioProfile", _mediaProfileSettings->getAudioProfileName(profile));
}

void MainWindowController::onVideoProfileChanged(uint cameraIndex, GStreamerUtil::VideoProfile profile)
{
    QMetaObject::invokeMethod(_window, "setVideoIsStreaming", Q_ARG(QVariant, cameraIndex), Q_ARG(QVariant, profile.codec != GStreamerUtil::CODEC_NULL));
    QMetaObject::invokeMethod(_window, "setVideoProfileName", Q_ARG(QVariant, cameraIndex), Q_ARG(QVariant, _mediaProfileSettings->getVideoProfileName(profile)));
}

} // namespace Soro
