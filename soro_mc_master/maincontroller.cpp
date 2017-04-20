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
#include <QMessageBox>

#include "libsoromc/constants.h"
#include "libsoromc/logger.h"

#include <ros/ros.h>

#define LogTag "MainController"

namespace Soro {

MainController *MainController::_self = nullptr;

MainController::MainController(QObject *parent) : QObject(parent) { }

void MainController::panic(QString tag, QString message)
{
    Logger::logError(LogTag, QString("panic(): %1: %2").arg(tag, message));
    QMessageBox::critical(0, "Mission Control", tag + ": " + message);
    Logger::logInfo(LogTag, "Committing suicide...");
    delete _self;
    Logger::logInfo(LogTag, "Exiting with code 1");
    exit(1);
}

void MainController::init(QApplication *app)
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
            // Load master settings
            //
            try {
                _self->_settings = new SettingsModel;
                _self->_settings->load();
            }
            catch (QString err)
            {
                panic(LogTag, QString("Error loading settings: %1").arg(err));
            }

            //
            // Create roscore controller, which will start roscore
            //
            Logger::logInfo(LogTag, "Initializing roscore controller...");
            _self->_roscoreController = new RosCoreController(_self);

            //
            // Initialize ROS
            //
            int argc = QCoreApplication::arguments().size();
            char *argv[argc];

            for (int i = 0; i < argc; i++) {
                argv[i] = QCoreApplication::arguments()[i].toLocal8Bit().data();
            }

            setenv("ROS_MASTER_URI", QString("http://localhost:%1").arg(SORO_NET_ROS_MASTER_PORT).toLatin1().constData(), 1);

            try
            {
                Logger::logInfo(LogTag, QString("Calling ros::init() with master URI \'%1\'").arg(getenv("ROS_MASTER_URI")));
                ros::init(argc, argv, getId().toStdString());
                Logger::logInfo(LogTag, "ROS initialized");
            }
            catch(ros::InvalidNameException e)
            {
                panic(LogTag, QString("Invalid name exception initializing ROS: %1").arg(e.what()));
                return;
            }

            //
            // Create master connection status controller
            //
            Logger::logInfo(LogTag, "Initializing master connection status controller...");
            _self->_masterConnectionStatusController = new MasterConnectionStatusController(_self->_settings, _self);

            //
            // Create broadcaster
            //
            Logger::logInfo(LogTag, "Initializing broadcaster...");
            _self->_broadcaster = new Broadcaster(_self);

            //
            // Create the media bouncer
            //
            Logger::logInfo(LogTag, "Initializing media bouncer...");
            _self->_mediaBouncer = new MediaBouncer(_self);

            //
            // Create the QML application engine
            //
            Logger::logInfo(LogTag, "Initializing QML engine...");
            _self->_qmlEngine = new QQmlEngine(_self);

            //
            // Create the main UI
            //
            Logger::logInfo(LogTag, "Creating main window...");
            _self->_mainWindowController = new MainWindowController(_self->_qmlEngine, _self);

            connect(_self->_masterConnectionStatusController, &MasterConnectionStatusController::connectedChanged,
                    _self->_mainWindowController, &MainWindowController::onConnectedChanged);
            connect(_self->_masterConnectionStatusController, &MasterConnectionStatusController::latencyUpdate,
                    _self->_mainWindowController, &MainWindowController::onLatencyUpdated);
            connect(_self->_masterConnectionStatusController, &MasterConnectionStatusController::bitrateUpdate,
                    _self->_mainWindowController, &MainWindowController::onBitrateUpdated);

            connect(_self->_mediaBouncer, &MediaBouncer::bitsRead,
                    _self->_masterConnectionStatusController, &MasterConnectionStatusController::logBitsDown);

            Logger::logInfo(LogTag, "Initialization complete");
        });
    }
}

//
// Getters
//

QString MainController::getId()
{
    return "mc_master";
}

} // namespace Soro
