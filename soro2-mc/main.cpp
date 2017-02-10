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

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication app(argc, argv);

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
        qCritical() << "Failed to create QML window (Reason given: " << qmlComponent.errorString() << ")";
        return 1;
    }

    //TODO

    return app.exec();
}
