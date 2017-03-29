#include "maincontroller.h"

#include <QTimer>
#include <QMessageBox>
#include <QtWebEngine>

#include <SDL2/SDL.h>

#include <Qt5GStreamer/QGst/Init>
#include <Qt5GStreamer/QGlib/Error>

#include "qquickgstreamersurface.h"

#include <ros/ros.h>

#define LogTag "MainController"

namespace Soro {

MainController *MainController::_self = nullptr;

MainController::MainController(QObject *parent) : QObject(parent) { }

void MainController::panic(QString message)
{
    logError(LogTag, QString("panic(): %1").arg(message));
    QMessageBox::critical(0, "Mission Control", message);
    QCoreApplication::exit(1);
}

void MainController::init(QApplication *app)
{
    if (_self)
    {
        logError(LogTag, "init() called when already initialized");
    }
    else
    {
        logInfo(LogTag, "Initializing");
        _self = new MainController(app);
        QTimer::singleShot(0, _self, &MainController::initInternal);
    }
}

void MainController::initInternal()
{
    //
    // Create a unique identifier for this mission control, it is mainly used as a unique node name for ROS
    //
    _mcId = genId();
    logInfo(LogTag, QString("Mission Control ID is: %1").arg(_mcId).toLocal8Bit().constData());

    //
    // Create the settings model and load the main settings file
    //
    logInfo(LogTag, "Loading settings...");
    try
    {
        _settingsModel = new SettingsModel;
        _settingsModel->load();
    }
    catch (QString err)
    {
        panic(QString("Error loading settings: %1").arg(err));
        return;
    }

    //
    // Create camera settings model to load camera configuration
    //
    logInfo(LogTag, "Loading camera settings...");
    try
    {
        _cameraSettingsModel = new CameraSettingsModel;
        _cameraSettingsModel->load();
    }
    catch (QString err)
    {
        panic(QString("Error loading camera settings: %1").arg(err));
        return;
    }

    //
    // Initialize Qt5GStreamer, must be called before anything else is done with it
    //
    logInfo(LogTag, "Initializing QtGstreamer...");
    try
    {
        QGst::init();
    }
    catch (QGlib::Error e)
    {
        panic(QString("Failed to initialize QtGStreamer:  %1").arg(e.message()));
        return;
    }

    //
    // Initiaize the Qt webengine (i.e. blink web engine) for use in QML
    //
    logInfo(LogTag, "Initializing QtWebEngine...");
    QtWebEngine::initialize();

    //
    // Initialize SDL (for gamepad reading)
    //
    logInfo(LogTag, "Initializing SDL...");
    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) != 0)
    {
        panic(QString("Failed to initialize SDL:  %1").arg(SDL_GetError()));
        return;
    }

    //
    // Load the SDL gamepad map file
    // This map file allows SDL to know which button/axis (i.e. "Left X Axis") corresponds
    // to the raw reading from the controller (i.e. "Axis 0")
    //
    logInfo(LogTag, "Initializing SDL gamepad map...");
    QFile gamepadMap(":/config/gamecontrollerdb.txt"); // Loads from assets.qrc
    gamepadMap.open(QIODevice::ReadOnly);
    while (gamepadMap.bytesAvailable())
    {
        if (SDL_GameControllerAddMapping(gamepadMap.readLine().data()) == -1)
        {
            panic(QString("Failed to apply SDL gamepad map: %1").arg(SDL_GetError()));
            gamepadMap.close();
            return;
        }
    }
    gamepadMap.close();

    //
    // Create the GamepadController instance
    //
    try
    {
        _gamepadController = new GamepadController(this);
    }
    catch (QString err)
    {
        panic(QString("Error initializing gamepad system: %1").arg(err));
        return;
    }

    //
    // Create ROS connection controller
    //
    try
    {
        _rosConnectionController = new RosConnectionController(this);
    }
    catch (QString err)
    {
        panic(QString("Error initializing ROS controller: %1").arg(err));
        return;
    }

    //
    // Create drive control system
    //
    try
    {
        _driveControlSystem = new DriveControlSystem(this);
    }
    catch (QString err)
    {
        panic(QString("Error initializing drive control system: %1").arg(err));
        return;
    }

    //
    // Create the QML application engine
    //
    logInfo(LogTag, "Initializing QML engine...");
    qmlRegisterType<QQuickGStreamerSurface>("Soro", 1, 0, "GStreamerSurface");
    _qmlEngine = new QQmlEngine(this);

    //
    // Create the main UI
    //
    try {
        _mainWindowController = new MainWindowController(_qmlEngine, this);
    }
    catch (QString err)
    {
        panic(QString("Error creating main window: %1").arg(err));
        return;
    }

    logInfo(LogTag, "Initialization complete");
}

void MainController::logDebug(QString tag, QString message)
{
    if (_self)
    {
        _self->log(LogLevelDebug, tag, message);
    }
}

void MainController::logInfo(QString tag, QString message)
{
    if (_self)
    {
        _self->log(LogLevelInfo, tag, message);
    }
}

void MainController::logWarning(QString tag, QString message)
{
    if (_self)
    {
        _self->log(LogLevelWarning, tag, message);
    }
}

void MainController::logError(QString tag, QString message)
{
    if (_self)
    {
        _self->log(LogLevelError, tag, message);
    }
}

void MainController::log(LogLevel level, QString tag, QString message) {
    if (ros::isInitialized()) {
        const char* formatted = QString("[%1] %2").arg(tag, message).toLocal8Bit().constData();
        switch (level) {
        case LogLevelDebug:
            ROS_DEBUG(formatted);
            break;
        case LogLevelInfo:
            ROS_INFO(formatted);
            break;
        case LogLevelWarning:
            ROS_WARN(formatted);
            break;
        case LogLevelError:
            ROS_ERROR(formatted);
            break;
        }
    }
    else {
        const char* formatted = QString("[No ROS] [%1] %2").arg(tag, message).toLocal8Bit().constData();
        switch (level) {
        case LogLevelDebug:
            qDebug(formatted);
            break;
        case LogLevelInfo:
            qInfo(formatted);
            break;
        case LogLevelWarning:
            qWarning(formatted);
            break;
        case LogLevelError:
            qCritical(formatted);
            break;
        }
    }
}

QString MainController::getMissionControlId()
{
    return _self->_mcId;
}

GamepadController* MainController::getGamepadController()
{
    return _self->_gamepadController;
}

SettingsModel* MainController::getSettingsModel()
{
    return _self->_settingsModel;
}

QString MainController::genId()
{
    const QString chars("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789");

    qsrand(QTime::currentTime().msec());
    QString randomString = "mc_";
    for(int i = 0; i < 12; ++i) {
        randomString.append(chars.at(qrand() % chars.length()));
    }
    return randomString;
}

} // namespace Soro
