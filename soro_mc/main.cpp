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

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QQuickStyle>
#include <QQmlComponent>
#include <QQmlEngine>
#include <QDebug>
#include <QQuickView>
#include <QQmlContext>
#include <QtWebEngine>
#include <QMessageBox>

#include <Qt5GStreamer/QGlib/Error>
#include <Qt5GStreamer/QGst/Init>

#include <SDL2/SDL.h>

#include <ros/ros.h>

#include <stdlib.h>

#include "settingsmodel.h"

using namespace Soro;

/* Generates a random string of letters and numbers, prefixed by 'mc_' to identify
 * this mission control.
 */
QString generateUniqueId()
{
    const QString chars("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789");

    qsrand(QTime::currentTime().msec());
    QString randomString = "mc_";
    for(int i = 0; i < 12; ++i) {
        randomString.append(chars.at(qrand() % chars.length()));
    }
    return randomString;
}

/* Called in the event of an unrecoverable init error
 */
void error(QString message)
{
    qCritical() << message;
    QMessageBox::critical(0, "Mission Control", message);
}

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setOrganizationName("Sooner Rover");
    QCoreApplication::setOrganizationDomain("ou.edu/soonerrover");
    QCoreApplication::setApplicationName("Mission Control");
    QApplication app(argc, argv);

    // Create a unique identifier for this mission control, it is mainly used as a unique node name for ROS
    QString mcId = generateUniqueId();

    // Create the settings model and load the main settings file
    SettingsModel settings;
    QString settingsErrorString;
    if (!settings.load(&settingsErrorString))
    {
        error(QString("<b>Failed to load settings:</b> %1").arg(settingsErrorString));
        return 1;
    }

    // Initialize Qt5GStreamer, must be called before anything else is done with it
    try
    {
        QGst::init(&argc, &argv);
    }
    catch (QGlib::Error e)
    {
        error(QString("<b>Failed to initialize QtGStreamer:</b>  %1").arg(e.message()));
        return 1;
    }

    // Initiaize the Qt webengine (i.e. blink web engine) for use in QML
    QtWebEngine::initialize();

    // Initialize SDL (for gamepad reading)
    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) != 0)
    {
        error(QString("<b>Failed to initialize SDL:</b>  %1").arg(SDL_GetError()));
        return 1;
    }

    // Load the SDL gamepad map file
    // This map file allows SDL to know which button/axis (i.e. "Left X Axis") corresponds
    // to the raw reading from the controller (i.e. "Axis 0")
    QFile gamepadMap(":/config/gamecontrollerdb.txt"); // Loads from assets.qrc
    gamepadMap.open(QIODevice::ReadOnly);
    while (gamepadMap.bytesAvailable())
    {
        if (SDL_GameControllerAddMapping(gamepadMap.readLine().data()) == -1)
        {
            error(QString("<b>Failed to apply SDL gamepad map:</b> %1").arg(SDL_GetError()));
            gamepadMap.close();
            return 1;
        }
    }
    gamepadMap.close();

    // Initialize ROS
    try
    {
        // ROS loads ROS_MASTER_URI through envvar, but we load it in the settings file
        setenv("ROS_MASTER_URI", settings.getRosMasterUri().toLatin1().data(), 1);
        ros::init(argc, argv, mcId.toStdString());
    }
    catch(ros::InvalidNameException e)
    {
        error(QString("<b>Failed to initialize ROS:</b> %1").arg(e.what()));
        return 1;
    }

    // TODO: ensure our mission control role is allowed (i.e. cannot have two arm mission controls)

    // Create the QML application engine
    QQmlEngine qmlEngine(&app);

    // Set the theme for the interface controls (like button, slider, radio, etc)
    // This only affects Quick Controls 2 elements
    QQuickStyle::setStyle("Material");

    // Create the main UI
    QQmlComponent qmlComponent(&qmlEngine, QUrl("qrc:/main.qml"));
    QQuickWindow *window = qobject_cast<QQuickWindow*>(qmlComponent.create());
    if (!qmlComponent.errorString().isEmpty() || !window)
    {
        // There was an error creating the QML window. This is most likely due to a QML syntax error
        error(QString("<b>Failed to create QML window:</b>  %1").arg(qmlComponent.errorString()));
        return 1;
    }

    //TODO

    return app.exec();
}
