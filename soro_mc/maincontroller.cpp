#include "maincontroller.h"

#include <QTimer>
#include <QMessageBox>
#include <QtWebEngine>

#include <SDL2/SDL.h>

#include <Qt5GStreamer/QGst/Init>
#include <Qt5GStreamer/QGlib/Error>

#include "qquickgstreamersurface.h"
#include "libsoromc/constants.h"
#include "libsoromc/logger.h"

#include <ros/ros.h>

#define LogTag "MainController"

namespace Soro {

MainController *MainController::_self = nullptr;

MainController::MainController(QObject *parent) : QObject(parent) { }

void MainController::panic(QString message)
{
    Logger::logError(LogTag, QString("panic(): %1").arg(message));
    QMessageBox::critical(0, "Mission Control", message);
    QCoreApplication::exit(1);
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
        panic(QString("Error loading settings: %1").arg(err));
        return;
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
        panic(QString("Error loading camera settings: %1").arg(err));
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
        panic(QString("Failed to initialize QtGStreamer:  %1").arg(e.message()));
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
        panic(QString("Failed to initialize SDL:  %1").arg(SDL_GetError()));
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
            panic(QString("Failed to apply SDL gamepad map: %1").arg(SDL_GetError()));
            gamepadMap.close();
            return;
        }
    }
    gamepadMap.close();

    //
    // Start master controller process
    //
    if (_settingsModel->getIsMaster()) {
        Logger::logInfo(LogTag, "Starting master controller process...");
        _masterController = new MasterController(this);

    }

    //
    // Setup ROS
    //
    Logger::logInfo(LogTag, "Initializing ROS...");
    try
    {
        if (_settingsModel->getIsMaster())
        {
            // We are the master mission control
            Logger::logInfo(LogTag, "This is not the master mission control, waiting for broadcast from master");
            setenv("ROS_MASTER_URI", QString("http://localhost:%1").arg(SORO_MC_ROS_MASTER_PORT).toLocal8Bit().constData(), 1);
            onRosMasterFound();
        }
        else
        {
            Logger::logInfo(LogTag, "This is the master mission control, starting ROS master");
            // We are not the master mission control, we should try to find the ros master
            _rosInitUdpSocket = new QUdpSocket(this);
            if (!_rosInitUdpSocket->bind(SORO_MC_BROADCAST_PORT))
            {
                MainController::panic(QString("Cannot bind to mission control UDP broadcast port: %1").arg(_rosInitUdpSocket->errorString()));
                return;
            }
            connect(_rosInitUdpSocket, SIGNAL(readyRead()), this, SLOT(rosInitUdpReadyRead()));
        }
    }
    catch (QString err)
    {
        panic(QString("Error initializing ROS controller: %1").arg(err));
        return;
    }

    // Setup will resume once ROS is initialized
}

void MainController::rosInitUdpReadyRead()
{
    while (_rosInitUdpSocket->hasPendingDatagrams())
    {
        char data[100];
        QHostAddress address;
        quint16 port;
        qint64 len = _rosInitUdpSocket->readDatagram(data, 100, &address, &port);

        if (strncmp(data, "master", qMax(strlen("master"), (size_t)len)) == 0)
        {
            // Received message from master mission control
            setenv("ROS_MASTER_URI", (QString("http://%1:%2").arg(address.toString(), SORO_MC_ROS_MASTER_PORT)).toLocal8Bit().constData(), 1);
            disconnect(_rosInitUdpSocket, SIGNAL(readyRead()), this, SLOT(rosInitUdpReadyRead()));
            delete _rosInitUdpSocket;
            _rosInitUdpSocket = nullptr;
            onRosMasterFound();
            break;
        }
        else
        {
            Logger::logError(LogTag, "Got invalid message on mission control broadcast port");
        }
    }
}

void MainController::onRosMasterFound() {

    //
    // Finish setting up ROS now that ROS_MASTER_URI is set
    //
    int argc = QCoreApplication::arguments().size();
    char *argv[argc];

    for (int i = 0; i < argc; i++) {
        argv[i] = QCoreApplication::arguments()[i].toLocal8Bit().data();
    }

    Logger::logInfo(LogTag, QString("Calling ros::init() with master URI \'%1\'").arg(getenv("ROS_MASTER_URI")));
    try
    {
        ros::init(argc, argv, MainController::getMissionControlId().toStdString());
        Logger::logInfo(LogTag, "Creating ROS NodeHandle...");
        _nodeHandle = new ros::NodeHandle;
        Logger::logInfo(LogTag, "ROS initialized");
    }
    catch(ros::InvalidNameException e)
    {
        panic(QString("Invalid name exception initializing ROS: %1").arg(e.what()));
        return;
    }

    //
    // Create connection status controller
    //
    Logger::logInfo(LogTag, "Initializing connection status controller...");
    try
    {
        _connectionStatusController = new ConnectionStatusController(this);
    }
    catch (QString err)
    {
        panic(QString("Error initializing connection status controller: %1").arg(err));
        return;
    }

    //
    // Create the GamepadController instance
    //
    Logger::logInfo(LogTag, "Initializing Gamepad Controller...");
    try
    {
        _gamepadController = new GamepadController(this);
    }
    catch (QString err)
    {
        panic(QString("Error initializing gamepad system: %1").arg(err));
        return;
    }

    switch (_settingsModel->getConfiguration()) {
    case SettingsModel::DriverConfiguration:
        //
        // Create drive control system
        //
        Logger::logInfo(LogTag, "Initializing drive control system...");
        try
        {
            _driveControlSystem = new DriveControlSystem(this);
        }
        catch (QString err)
        {
            panic(QString("Error initializing drive control system: %1").arg(err));
            return;
        }
        break;
    case SettingsModel::ArmOperatorConfiguration:
        break;
    case SettingsModel::CameraOperatorConfiguration:
        break;
    case SettingsModel::ObserverConfiguration:
        break;
    }

    //
    // Create the QML application engine
    //
    Logger::logInfo(LogTag, "Initializing QML engine...");
    qmlRegisterType<QQuickGStreamerSurface>("Soro", 1, 0, "GStreamerSurface");
    _qmlEngine = new QQmlEngine(this);

    //
    // Create the main UI
    //
    Logger::logInfo(LogTag, "Creating main window...");
    try {
        _mainWindowController = new MainWindowController(_qmlEngine, this);
    }
    catch (QString err)
    {
        panic(QString("Error creating main window: %1").arg(err));
        return;
    }

    Logger::logInfo(LogTag, "Initialization complete");
    emit initialized();
}

//
// Getters
//

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

CameraSettingsModel* MainController::getCameraSettingsModel()
{
    return _self->_cameraSettingsModel;
}

ConnectionStatusController* MainController::getConnectionStatusController()
{
    return _self->_connectionStatusController;
}

ros::NodeHandle* MainController::getNodeHandle() {
    return _self->_nodeHandle;
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
        id = "mc_observer";
        for(int i = 0; i < 12; ++i) {
            id.append(chars.at(qrand() % chars.length()));
        }
        return id;
    }

    return id;
}

} // namespace Soro
