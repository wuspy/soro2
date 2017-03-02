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

MainController::MainController(QObject *parent) : QObject(parent)
{
}

void MainController::panic(QString message)
{
    logFatal(LogTag, QString("panic(): %1").arg(message));
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
    _settingsModel = new SettingsModel;
    if (!_settingsModel->load())
    {
        panic(QString("Failed to load settings file: %1").arg(_settingsModel->errorString()));
        return;
    }

    //
    // Create camera settings model to load camera configuration
    //
    logInfo(LogTag, "Loading camera settings...");
    _cameraSettingsModel = new CameraSettingsModel;
    if (!_cameraSettingsModel->load())
    {
        panic(QString("Failed to load camera settings file: %1").arg(_cameraSettingsModel->errorString()));
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
    // Create the QML application engine
    //
    logInfo(LogTag, "Initializing QML engine...");
    qmlRegisterType<QQuickGStreamerSurface>("Soro", 1, 0, "GStreamerSurface");
    _qmlEngine = new QQmlEngine(this);

    //
    // Create the GamepadController instance
    //
    _gamepadController = new GamepadController(this);

    //
    // Create ROS connection controller
    //
    _rosConnectionController = new RosConnectionController(this);

    //
    // Create drive control system
    //
    _driveControlSystem = new DriveControlSystem(this);

    //
    // Create the main UI
    //
    QQmlComponent qmlComponent(_qmlEngine, QUrl("qrc:/main.qml"));
    QQuickWindow *window = qobject_cast<QQuickWindow*>(qmlComponent.create());
    if (!qmlComponent.errorString().isEmpty() || !window)
    {
        // There was an error creating the QML window. This is most likely due to a QML syntax error
        panic(QString("Failed to create QML window:  %1").arg(qmlComponent.errorString()));
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

void MainController::logFatal(QString tag, QString message)
{
    if (_self)
    {
        _self->log(LogLevelFatal, tag, message);
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
        case LogLevelFatal:
            ROS_FATAL(formatted);
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
        case LogLevelFatal:
            qFatal(formatted);
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
