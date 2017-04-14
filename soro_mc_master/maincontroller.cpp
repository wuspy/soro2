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
    // Create roscore controller, which will start roscore
    //
    Logger::logInfo(LogTag, "Initializing roscore controller...");
    try
    {
        _roscoreController = new RosCoreController(this);
    }
    catch (QString err)
    {
        panic(QString("Error initializing roscore controller: %1").arg(err));
        return;
    }

    //
    // Initialize ROS
    //
    int argc = QCoreApplication::arguments().size();
    char *argv[argc];

    for (int i = 0; i < argc; i++) {
        argv[i] = QCoreApplication::arguments()[i].toLocal8Bit().data();
    }

    setenv("ROS_MASTER_URI", QString("http://localhost:%1").arg(SORO_MC_ROS_MASTER_PORT).toLatin1().constData(), 1);

    Logger::logInfo(LogTag, QString("Calling ros::init() with master URI \'%1\'").arg(getenv("ROS_MASTER_URI")));
    try
    {
        ros::init(argc, argv, getId().toStdString());
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
    // Create master connection status controller
    //
    Logger::logInfo(LogTag, "Initializing master connection status controller...");
    try
    {
        _masterConnectionStatusController = new MasterConnectionStatusController(this);
    }
    catch (QString err)
    {
        panic(QString("Error initializing master connection status controller: %1").arg(err));
        return;
    }

    //
    // Create broadcaster
    //
    Logger::logInfo(LogTag, "Initializing broadcaster...");
    try
    {
        _broadcaster = new Broadcaster(this);
    }
    catch (QString err)
    {
        panic(QString("Error initializing broadcaster: %1").arg(err));
        return;
    }

    //
    // Create the QML application engine
    //
    Logger::logInfo(LogTag, "Initializing QML engine...");
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
    Q_EMIT initialized();
}

//
// Getters
//

QString MainController::getId()
{
    return "mc_master";
}

ros::NodeHandle* MainController::getNodeHandle()
{
    return _self->_nodeHandle;
}

MasterConnectionStatusController* MainController::getMasterConnectionStatusController()
{
    return _self->_masterConnectionStatusController;
}

RosCoreController* MainController::getRoscoreController()
{
    return _self->_roscoreController;
}

Broadcaster* MainController::getBroadcaster()
{
    return _self->_broadcaster;
}

} // namespace Soro
