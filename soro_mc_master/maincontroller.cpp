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

#include "soro_core/constants.h"
#include "soro_core/logger.h"

#include <ros/ros.h>

#define LogTag "MainController"

namespace Soro {

MainController *MainController::_self = nullptr;

MainController::MainController(QObject *parent) : QObject(parent) { }

void MainController::panic(QString tag, QString message)
{
    Logger::logError(LogTag, QString("panic(): %1: %2").arg(tag, message));
    QMessageBox::critical(0, "Mission Control", "<b>Fatal error in " + tag + "</b><br><br>" + message);
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
            try
            {
                _self->_settings = new SettingsModel;
                _self->_settings->load();
            }
            catch (QString err)
            {
                panic(LogTag, QString("Error loading settings: %1").arg(err));
            }

            //
            // Create camera settings model to load camera configuration
            //
            Logger::logInfo(LogTag, "Loading camera settings...");
            try
            {
                _self->_cameraSettings = new CameraSettingsModel;
                _self->_cameraSettings->load();
            }
            catch (QString err)
            {
                panic(LogTag, QString("Error loading camera settings: %1").arg(err));
                return;
            }

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
                return;
            }

            //
            // Create ros node list
            //
            Logger::logInfo(LogTag, "Initializing ros node list...");
            _self->_rosNodeList = new RosNodeList(1000, _self);

            //
            // Create master connection status controller
            //
            Logger::logInfo(LogTag, "Initializing master connection status controller...");
            _self->_masterConnectionStatusController = new MasterConnectionStatusController(_self->_settings, _self);

            //
            // Create the master video controller
            //
            Logger::logInfo(LogTag, "Initializing media bouncer...");
            _self->_masterVideoController = new MasterVideoController(_self->_cameraSettings, _self->_rosNodeList, _self);

            //
            // Create the QML application engine
            //
            Logger::logInfo(LogTag, "Initializing QML engine...");
            _self->_qmlEngine = new QQmlEngine(_self);

            //
            // Create the main UI
            //
            Logger::logInfo(LogTag, "Creating main window...");
            _self->_mainWindowController = new MainWindowController(_self->_qmlEngine, _self->_rosNodeList, _self);

            connect(_self->_masterConnectionStatusController, &MasterConnectionStatusController::connectedChanged,
                    _self->_mainWindowController, &MainWindowController::onConnectedChanged);
            connect(_self->_masterConnectionStatusController, &MasterConnectionStatusController::latencyUpdate,
                    _self->_mainWindowController, &MainWindowController::onLatencyUpdated);
            connect(_self->_masterConnectionStatusController, &MasterConnectionStatusController::bitrateUpdate,
                    _self->_mainWindowController, &MainWindowController::onBitrateUpdated);

            connect(_self->_masterVideoController, &MasterVideoController::bitsDown,
                    _self->_masterConnectionStatusController, &MasterConnectionStatusController::logBitsDown);

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
    return "mc_master";
}

} // namespace Soro
