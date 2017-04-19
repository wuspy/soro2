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
        QTimer::singleShot(0, _self, &MainController::initInternal);
    }
}

void MainController::initInternal()
{
    //
    // Create the settings model and load the main settings file
    //
    Logger::logInfo(LogTag, "Loading settings...");
    try
    {
        _settingsModel = new SettingsModel;
        _settingsModel->load();
    }
    catch (QString err)
    {
        panic(LogTag, QString("Error loading settings: %1").arg(err));
    }

    //
    // Create a unique identifier for this mission control, it is mainly used as a unique node name for ROS
    //
    _mcId = genId();
    Logger::logInfo(LogTag, QString("Mission Control ID is: %1").arg(_mcId).toLocal8Bit().constData());

    //
    // Create camera settings model to load camera configuration
    //
    Logger::logInfo(LogTag, "Loading camera settings...");
    try
    {
        _cameraSettingsModel = new CameraSettingsModel;
        _cameraSettingsModel->load();
    }
    catch (QString err)
    {
        panic(LogTag, QString("Error loading camera settings: %1").arg(err));
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

    //
    // Create master locator to find the ROS master address
    //
    Logger::logInfo(LogTag, "Initializing master locator...");
    _masterLocator = new MasterLocator(this);
    Logger::logInfo(LogTag, "Waiting for broadcast from master before continuing...");
    connect(_masterLocator, &MasterLocator::masterFound, this, &MainController::onRosMasterFound);
}

void MainController::onRosMasterFound(QHostAddress address)
{
    QString masterUrl = QString("http://%1:%2").arg(address.toString(), QString::number(SORO_NET_ROS_MASTER_PORT));
    setenv("ROS_MASTER_URI", masterUrl.toLatin1().constData(), 1);

    //
    // Finish setting up ROS now that ROS_MASTER_URI is set
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
    // Delete master locator now that we are done with it
    //
    disconnect(_masterLocator, &MasterLocator::masterFound, this, &MainController::onRosMasterFound);

    //
    // Create connection status controller
    //
    Logger::logInfo(LogTag, "Initializing connection status controller...");
    _connectionStatusController = new ConnectionStatusController(this);

    //
    // Create the GamepadController instance
    //
    Logger::logInfo(LogTag, "Initializing Gamepad Controller...");
    _gamepadController = new GamepadController(this);

    switch (_settingsModel->getConfiguration()) {
    case SettingsModel::DriverConfiguration:
        //
        // Create drive control system
        //
        Logger::logInfo(LogTag, "Initializing drive control system...");
        _driveControlSystem = new DriveControlSystem(_settingsModel->getDriveSendInterval(),
                                                     _gamepadController,
                                                     _connectionStatusController,
                                                     this);
        break;
    case SettingsModel::ArmOperatorConfiguration:
        //
        // Create arm control system
        //
        Logger::logInfo(LogTag, "Initializing arm control system...");
        _armControlSystem = new ArmControlSystem(this);
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
    _audioController = new AudioController(this);

    //
    // Create the QML application engine and setup the GStreamer surface
    //
    if (_settingsModel->getEnableHwRendering())
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
    _qmlEngine = new QQmlEngine(this);

    //
    // Create the main UI
    //
    Logger::logInfo(LogTag, "Creating main window...");
    _mainWindowController = new MainWindowController(_qmlEngine, _settingsModel, _cameraSettingsModel, this);

    //
    // Create the video controller instance
    //
    Logger::logInfo(LogTag, "Initializing video controller...");
    _videoController = new VideoController(_settingsModel, _cameraSettingsModel, _mainWindowController->getVideoSinks(), this);

    connect(_videoController, &VideoController::error,
            this, &MainController::onVideoError);
    connect(_videoController, &VideoController::eos,
            this, &MainController::onVideoEos);
    connect(_audioController, &AudioController::error,
            this, &MainController::onAudioError);
    connect(_audioController, &AudioController::eos,
            this, &MainController::onAudioEos);

    connect(_gamepadController, &GamepadController::buttonPressed,
            _mainWindowController, &MainWindowController::onGamepadButtonPressed);
    connect(_connectionStatusController, &ConnectionStatusController::bitrateUpdate,
            _mainWindowController, &MainWindowController::onBitrateUpdated);
    connect(_connectionStatusController, &ConnectionStatusController::latencyUpdate,
            _mainWindowController, &MainWindowController::onLatencyUpdated);
    connect(_connectionStatusController, &ConnectionStatusController::connectedChanged,
            _mainWindowController, &MainWindowController::onConnectedChanged);

    Logger::logInfo(LogTag, "Initialization complete");
}

void MainController::onVideoError(QString message, uint cameraIndex)
{
    _mainWindowController->notify(NOTIFICATION_TYPE_ERROR,
                                  "Error playing " + _cameraSettingsModel->getCamera(cameraIndex).name,
                                  "There was an error while decoding the video stream: " + message);
}

void MainController::onAudioError(QString message)
{
    _mainWindowController->notify(NOTIFICATION_TYPE_ERROR,
                                  "Error playing audio",
                                  "There was an error while decoding the audio stream: " + message);
}

void MainController::onVideoEos(uint cameraIndex)
{
    _mainWindowController->notify(NOTIFICATION_TYPE_ERROR,
                                  "Error playing " + _cameraSettingsModel->getCamera(cameraIndex).name,
                                  "Received end-of-stream message while streaming video");
}

void MainController::onAudioEos()
{
    _mainWindowController->notify(NOTIFICATION_TYPE_ERROR,
                                  "Error playing audio",
                                  "Received end-of-stream message while streaming audio");
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
