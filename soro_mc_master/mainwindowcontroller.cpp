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

#include <QQmlComponent>

#define LogTag "MainWindowController"

namespace Soro {

MainWindowController::MainWindowController(QQmlEngine *engine, const RosNodeList *rosNodeList, QObject *parent) : QObject(parent)
{
    _rosNodeList = rosNodeList;
    connect(_rosNodeList, &RosNodeList::nodesUpdated, this, &MainWindowController::onRosNodeListUpdated);

    QQmlComponent qmlComponent(engine, QUrl("qrc:/main.qml"));
    _window = qobject_cast<QQuickWindow*>(qmlComponent.create());
    if (!qmlComponent.errorString().isEmpty() || !_window)
    {
        // There was an error creating the QML window. This is most likely due to a QML syntax error
        MainController::panic(LogTag, QString("Failed to create QML window:  %1").arg(qmlComponent.errorString()));
    }
}

void MainWindowController::onRosNodeListUpdated()
{
    QString nodes = "<ul>";
    for (RosNodeList::Node node : _rosNodeList->getNodes())
    {
        nodes += QString("<li><b>%1</b> at %2</li>").arg(node.name, node.address.toString());
    }
    nodes += "</ul>";
    _window->setProperty("connectedNodeInfo", nodes);
}

void MainWindowController::onConnectedChanged(bool connected)
{
    _window->setProperty("connected", connected);
}

void MainWindowController::onLatencyUpdated(quint32 latency)
{
    _window->setProperty("latency", latency);
}

void MainWindowController::onDataRateUpdated(quint64 rateUp, quint64 rateDown)
{
    _window->setProperty("dataRateUp", rateUp);
    _window->setProperty("dataRateDown", rateDown);
}

} // namespace Soro
