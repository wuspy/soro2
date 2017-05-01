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
#include <QtWebEngine>

#include <SDL2/SDL.h>

#include <Qt5GStreamer/QGst/Init>
#include <Qt5GStreamer/QGlib/Error>

#include "qmlgstreamerglitem.h"
#include "qmlgstreamerpainteditem.h"
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

            //
            // Create a unique identifier for this mission control, it is mainly used as a unique node name for ROS
            //
            _self->_mcId = _self->genId();
            Logger::logInfo(LogTag, QString("Mission Control ID is: %1").arg(_self->_mcId).toLocal8Bit().constData());

            //
            // Create camera settings model to load camera configuration
            //
            Logger::logInfo(LogTag, "Loading camera settings...");
            try
            {
                _self->_cameraSettingsModel = new CameraSettingsModel;
                _self->_cameraSettingsModel->load();
            }
            catch (QString err)
            {
                panic(LogTag, QString("Error loading camera settings: %1").arg(err));
                return;
            }

            //
            // Create media settings model to load audio/video profile configuration
            //
            Logger::logInfo(LogTag, "Loading media profile settings...");
            try
            {
                _self->_mediaProfileSettingsModel = new MediaProfileSettingsModel;
                _self->_mediaProfileSettingsModel->load();
            }
            catch (QString err)
            {
                panic(LogTag, QString("Error loading media profile settings: %1").arg(err));
                return;
            }

            //
            // Create media settings model to load keyboard & gamepad bindings
            //
            Logger::logInfo(LogTag, "Loading keyboard and gamepad binding profile settings...");
            try
            {
                _self->_bindsSettingsModel = new BindsSettingsModel;
                _self->_bindsSettingsModel->load();
            }
            catch (QString err)
            {
                panic(LogTag, QString("Error loading keyboard and gamepad binding profile settings: %1").arg(err));
                return;
            }

            //
            // Initialize Qt5GStreamer, must be called before anything else is done with it
            //
            Logger::logInfo(LogTag, "Initializing QtGstreamer...");
            try
            {
                QGst::init();
            }
            catch (QGlib::Error e)
            {
                panic(LogTag, QString("Failed to initialize QtGStreamer:  %1").arg(e.message()));
                return;
            }

            //
            // Initiaize the Qt webengine (i.e. blink web engine) for use in QML
            //
            Logger::logInfo(LogTag, "Initializing QtWebEngine...");
            QtWebEngine::initialize();

            //
            // Initialize SDL (for gamepad reading)
            //
            Logger::logInfo(LogTag, "Initializing SDL...");
            if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) != 0)
            {
                panic(LogTag, QString("Failed to initialize SDL:  %1").arg(SDL_GetError()));
                return;
            }

            //
            // Load the SDL gamepad map file
            // This map file allows SDL to know which button/axis (i.e. "Left X Axis") corresponds
            // to the raw reading from the controller (i.e. "Axis 0")
            //
            Logger::logInfo(LogTag, "Initializing SDL gamepad map...");
            QFile gamepadMap(":/config/gamecontrollerdb.txt"); // Loads from assets.qrc
            gamepadMap.open(QIODevice::ReadOnly);
            while (gamepadMap.bytesAvailable())
            {
                if (SDL_GameControllerAddMapping(gamepadMap.readLine().data()) == -1)
                {
                    panic(LogTag, QString("Failed to apply SDL gamepad map: %1").arg(SDL_GetError()));
                    gamepadMap.close();
                    return;
                }
            }
            gamepadMap.close();

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
                ros::init(argc, argv, MainController::getId().toStdString());
                Logger::logInfo(LogTag, "ROS initialized");
            }
            catch(ros::InvalidNameException e)
            {
                panic(LogTag, QString("Invalid name exception initializing ROS: %1").arg(e.what()));
                return;
            }

            //
            // Create connection status controller
            //
            Logger::logInfo(LogTag, "Initializing connection status controller...");
            _self->_connectionStatusController = new ConnectionStatusController(_self);

            //
            // Create the GamepadController instance
            //
            Logger::logInfo(LogTag, "Initializing Gamepad Controller...");
            _self->_gamepadController = new GamepadController(_self);

            switch (_self->_settingsModel->getConfiguration()) {
            case SettingsModel::DriverConfiguration:
                //
                // Create drive control system
                //
                Logger::logInfo(LogTag, "Initializing drive control system...");
                _self->_driveControlSystem = new DriveControlSystem(_self->_settingsModel->getDriveSendInterval(),
                                                             _self->_gamepadController,
                                                             _self->_connectionStatusController,
                                                             _self);
                _self->_driveControlSystem->setLimit(_self->_settingsModel->getDrivePowerLimit());
                _self->_driveControlSystem->setSkidSteerFactor(_self->_settingsModel->getDriveSkidSteerFactor());
                break;
            case SettingsModel::ArmOperatorConfiguration:
                //
                // Create arm control system
                //
                Logger::logInfo(LogTag, "Initializing arm control system...");
                _self->_armControlSystem = new ArmControlSystem(_self);
                break;
            case SettingsModel::CameraOperatorConfiguration:
                //TODO
                break;
            case SettingsModel::ObserverConfiguration:
                break;
            }

            //
            // Create the audio controller instance
            //
            Logger::logInfo(LogTag, "Initializing audio controller...");
            _self->_audioController = new AudioController(_self);

            //
            // Create the mission control broadcaster
            //
            Logger::logInfo(LogTag, "Initializing mission control broadcaster...");
            try
            {
                _self->_mcBroadcaster = new Broadcaster(SORO_NET_MC_BROADCAST_PORT, getId(), 1000, _self);
            }
            catch (QString message)
            {
                panic(LogTag, "Error initializing mission control broadcast: " + message);
            }


            //
            // Create the QML application engine and setup the GStreamer surface
            //
            if (_self->_settingsModel->getEnableHwRendering())
            {
                // Use the hardware opengl rendering surface, doesn't work on some hardware
                Logger::logInfo(LogTag, "Registering QmlGStreamerItem as GStreamerSurface...");
                qmlRegisterType<QmlGStreamerGlItem>("Soro", 1, 0, "GStreamerSurface");
            }
            else
            {
                // Use the software rendering surface, works everywhere but slower
                Logger::logInfo(LogTag, "Registering QmlGStreamerPaintedItem as GStreamerSurface...");
                qmlRegisterType<QmlGStreamerPaintedItem>("Soro", 1, 0, "GStreamerSurface");
            }
            Logger::logInfo(LogTag, "Initializing QML engine...");
            _self->_qmlEngine = new QQmlEngine(_self);

            //
            // Create the main UI
            //
            Logger::logInfo(LogTag, "Creating main window...");
            _self->_mainWindowController = new MainWindowController(_self->_qmlEngine, _self->_settingsModel, _self->_cameraSettingsModel, _self);

            //
            // Create the video controller instance
            //
            Logger::logInfo(LogTag, "Initializing video controller...");
            _self->_videoController = new VideoController(_self->_settingsModel, _self->_cameraSettingsModel, _self->_mainWindowController->getVideoSinks(), _self);


            // Forward audio/video errors to the UI
            connect(_self->_videoController, &VideoController::gstError, _self, [](QString message, uint cameraIndex)
            {
                _self->_self->_mainWindowController->notify(NOTIFICATION_TYPE_ERROR,
                                              "Error playing " + _self->_cameraSettingsModel->getCamera(cameraIndex).name,
                                              "There was an error while decoding the video stream: " + message);
            });
            connect(_self->_videoController, &VideoController::gstEos, _self, [](uint cameraIndex)
            {
                _self->_mainWindowController->notify(NOTIFICATION_TYPE_ERROR,
                                              "Error playing " + _self->_cameraSettingsModel->getCamera(cameraIndex).name,
                                              "Received end-of-stream message while streaming video");
            });
            connect(_self->_audioController, &AudioController::gstError, _self, [](QString message)
            {
                _self->_mainWindowController->notify(NOTIFICATION_TYPE_ERROR,
                                              "Error playing audio",
                                              "There was an error while decoding the audio stream: " + message);
            });
            connect(_self->_audioController, &AudioController::gstEos, _self, []()
            {
                _self->_mainWindowController->notify(NOTIFICATION_TYPE_ERROR,
                                              "Error playing audio",
                                              "Received end-of-stream message while streaming audio");
            });

            // Connect gamepad, bitrate, and status events to the UI
            connect(_self->_gamepadController, &GamepadController::buttonPressed,
                    _self->_mainWindowController, &MainWindowController::onGamepadButtonPressed);
            connect(_self->_connectionStatusController, &ConnectionStatusController::bitrateUpdate,
                    _self->_mainWindowController, &MainWindowController::onBitrateUpdated);
            connect(_self->_connectionStatusController, &ConnectionStatusController::latencyUpdate,
                    _self->_mainWindowController, &MainWindowController::onLatencyUpdated);
            connect(_self->_connectionStatusController, &ConnectionStatusController::connectedChanged,
                    _self->_mainWindowController, &MainWindowController::onConnectedChanged);


            // Temporary implementation for a 'turbo' button
            connect(_self->_gamepadController, &GamepadController::buttonPressed, _self, [](SDL_GameControllerButton btn, bool isPressed)
            {
               if (btn == SDL_CONTROLLER_BUTTON_LEFTSHOULDER)
               {
                   _self->_driveControlSystem->setLimit(isPressed ? 1.0 : 0.6);
               }
            });

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
    return _self->_mcId;
}

//
// Misc private functions
//

QString MainController::genId()
{
    const QString chars("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789");

    qsrand(QTime::currentTime().msec());
    QString id;
    switch (_settingsModel->getConfiguration())
    {
    case SettingsModel::ArmOperatorConfiguration:
        id = "mc_arm";
        break;
    case SettingsModel::DriverConfiguration:
        id = "mc_drive";
        break;
    case SettingsModel::CameraOperatorConfiguration:
        id = "mc_camera";
        break;
    case SettingsModel::ObserverConfiguration:
        id = "mc_observer_";
        for(int i = 0; i < 12; ++i) {
            id.append(chars.at(qrand() % chars.length()));
        }
        return id;
    }

    return id;
}

} // namespace Soro
