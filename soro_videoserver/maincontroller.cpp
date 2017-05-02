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

#include "maincontroller.h"

#include <QTimer>

#include <ros/ros.h>

#include "soro_core/constants.h"
#include "soro_core/logger.h"

#define LogTag "MainController"

namespace Soro {

MainController *MainController::_self = nullptr;

MainController::MainController(QObject *parent) : QObject(parent) { }

void MainController::panic(QString tag, QString message)
{
    Logger::logError(LogTag, QString("panic(): %1: %2").arg(tag, message));
    Logger::logInfo(LogTag, "Committing suicide...");
    delete _self;
    Logger::logInfo(LogTag, "Exiting with code 1");
    exit(1);
}

void MainController::init(QCoreApplication *app)
{
    if (_self)
    {
        Logger::logError(LogTag, "init() called when already initialized");
    }
    else
    {
        Logger::logInfo(LogTag, "Starting...");
        _self = new MainController(app);

        // Use a timer to wait for the event loop to start
        QTimer::singleShot(0, _self, []()
        {
            //
            // Connect to D-Bus
            //
            if (!QDBusConnection::sessionBus().isConnected()) {
                Logger::logError(LogTag, "Cannot connect to D-Bus session bus");
                return 1;
            }

            if (!QDBusConnection::sessionBus().registerService(SORO_DBUS_VIDEO_PARENT_SERVICE_NAME)) {
                Logger::logError(LogTag, "Cannot register D-Bus service: " + QDBusConnection::sessionBus().lastError().message());
                return 1;
            }

            //
            // Create the settings model and load the main settings file
            //
            Logger::logInfo(LogTag, "Loading settings...");
            try
            {
                _self->_settingsModel = new SettingsModel;
                _self->_settingsModel->load();
            }
            catch (QString err)
            {
                panic(LogTag, QString("Error loading settings: %1").arg(err));
            }

            _self->_id = "video_server_" + QString::number(_self->_settingsModel->getComputerIndex());

            // Make sure ROS_MASTER_URI is set
            if (getenv("ROS_MASTER_URI") == nullptr)
            {
                panic(LogTag, "ROS_MASTER_URI is not set in the environment");
            }
            Logger::logInfo(LogTag, "ROS_MASTER_URI is at " + QString(getenv("ROS_MASTER_URI")));

            //
            // Initialize ROS
            //
            int argc = QCoreApplication::arguments().size();
            char *argv[argc];

            for (int i = 0; i < argc; i++) {
                argv[i] = QCoreApplication::arguments()[i].toLocal8Bit().data();
            }

            try
            {
                Logger::logInfo(LogTag, QString("Calling ros::init() with master URI \'%1\'").arg(getenv("ROS_MASTER_URI")));
                ros::init(argc, argv, getId().toStdString());
                Logger::logInfo(LogTag, "ROS initialized");
            }
            catch(ros::InvalidNameException e)
            {
                panic(LogTag, QString("Invalid name exception initializing ROS: %1").arg(e.what()));
            }

            //
            // Initialize ros node list
            //
            Logger::logInfo(LogTag, "Initializing ros node list...");
            _self->_rosNodeList = new RosNodeList(1000, _self);

            //
            // Create video server
            //
            Logger::logInfo(LogTag, "Initializing video server...");
            _self->_videoServer = new VideoServer(_self->_settingsModel->getComputerIndex(), _self->_rosNodeList, _self);

            // Start ROS spinner loop
            _self->_rosSpinTimerId = _self->startTimer(1);

            Logger::logInfo(LogTag, "Initialization complete");
        });
    }
}

void MainController::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _rosSpinTimerId)
    {
        ros::spinOnce();
    }
}

//
// Getters
//

QString MainController::getId()
{
    return _self->_id;
}

} // namespace Soro
