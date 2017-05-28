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

#include "maincontroller.h"

#include <QTimer>
#include <QMessageBox>

#include "soro_core/constants.h"
#include "soro_core/logger.h"

#define LogTag "MainController"

namespace Soro {

MainController *MainController::_self = nullptr;

MainController::MainController(QObject *parent) : QObject(parent) { }

void MainController::panic(QString tag, QString message)
{
    LOG_E(LogTag, QString("panic(): %1: %2").arg(tag, message));
    QMessageBox::critical(0, "Mission Control", "<b>Fatal error in " + tag + "</b><br><br>" + message);
    LOG_I(LogTag, "Committing suicide...");
    delete _self;
    LOG_I(LogTag, "Exiting with code 1");
    exit(1);
}

void MainController::init(QApplication *app)
{
    if (_self)
    {
        LOG_E(LogTag, "init() called when already initialized");
    }
    else
    {
        LOG_I(LogTag, "Starting...");
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
            LOG_I(LogTag, "Loading camera settings...");
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

            //
            // Create master connection status controller
            //
            LOG_I(LogTag, "Initializing master connection status controller...");
            _self->_masterConnectionStatusController = new MasterConnectionStatusController(_self->_settings, _self);

            //
            // Create the master video controller
            //
            LOG_I(LogTag, "Initializing master video controller...");
            _self->_masterVideoController = new MasterVideoClient(_self->_settings, _self->_cameraSettings, _self);

            //
            // Create the master audio controller
            //
            LOG_I(LogTag, "Initializing master audio controller...");
            _self->_masterAudioController = new MasterAudioController(_self->_settings, _self);

            //
            // Create the QML application engine
            //
            LOG_I(LogTag, "Initializing QML engine...");
            _self->_qmlEngine = new QQmlEngine(_self);

            //
            // Create the main UI
            //
            LOG_I(LogTag, "Creating main window...");
            _self->_mainWindowController = new MainWindowController(_self->_qmlEngine, _self);

            connect(_self->_masterConnectionStatusController, &MasterConnectionStatusController::mqttConnected,
                    _self->_mainWindowController, &MainWindowController::onConnected);
            connect(_self->_masterConnectionStatusController, &MasterConnectionStatusController::mqttDisconnected,
                    _self->_mainWindowController, &MainWindowController::onDisconnected);
            connect(_self->_masterConnectionStatusController, &MasterConnectionStatusController::latencyUpdate,
                    _self->_mainWindowController, &MainWindowController::onLatencyUpdated);
            connect(_self->_masterConnectionStatusController, &MasterConnectionStatusController::dataRateUpdate,
                    _self->_mainWindowController, &MainWindowController::onDataRateUpdated);

            connect(_self->_masterVideoController, &MasterVideoClient::bytesDown,
                    _self->_masterConnectionStatusController, &MasterConnectionStatusController::logDataFromRover);
            connect(_self->_masterAudioController, &MasterAudioController::bytesDown,
                    _self->_masterConnectionStatusController, &MasterConnectionStatusController::logDataFromRover);

            connect(_self->_masterVideoController, &MasterVideoClient::bounceAddressesChanged,
                    _self->_mainWindowController, &MainWindowController::onVideoBounceAddressesChanged);
            connect(_self->_masterAudioController, &MasterAudioController::bounceAddressesChanged,
                    _self->_mainWindowController, &MainWindowController::onAudioBounceAddressesChanged);

            LOG_I(LogTag, "Initialization complete");
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
