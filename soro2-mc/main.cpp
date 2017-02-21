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

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QQuickStyle>
#include <QQmlComponent>
#include <QQmlEngine>
#include <QDebug>
#include <QQuickView>
#include <QQmlContext>
#include <QtWebEngine>

#include <Qt5GStreamer/QGlib/Error>
#include <Qt5GStreamer/QGst/Init>

#include <SDL2/SDL.h>

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);

    // Initialize Qt5GStreamer, must be called before anything else is done with it
    try
    {
        QGst::init(&argc, &argv);
    }
    catch (QGlib::Error error)
    {
        QMessageBox::critical(0, "Mission Control", QString("Failed to initialize QtGStreamer (Reason given:  %1)").arg(error.message()));
        return 1;
    }

    // Initiaize the Qt webengine (i.e. blink web engine) for use in QML
    QtWebEngine::initialize();

    // Initialize SDL (for gamepad reading)
    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) != 0)
    {
        QMessageBox::critical(0, "Mission Control", QString("Failed to initialize SDL (Reason given:  %1)").arg(SDL_GetError()));
        return 1;
    }

    // Create the QML application engine
    QQmlEngine qmlEngine(&app);

    // Set the theme for the interface controls (like button, slider, radio, etc)
    QQuickStyle::setStyle("Material");

    // Create the main UI
    QQmlComponent qmlComponent(&qmlEngine, QUrl("qrc:/main.qml"));
    QQuickWindow *window = qobject_cast<QQuickWindow*>(qmlComponent.create());
    if (!qmlComponent.errorString().isEmpty() || !window)
    {
        // There was an error creating the QML window. This is most likely due to a QML syntax error
        QMessageBox::critical(0, "Mission Control", QString("Failed to create QML window (Reason given:  %1)").arg(qmlComponent.errorString()));
        return 1;
    }

    //TODO

    return app.exec();
}
