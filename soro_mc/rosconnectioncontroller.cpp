#include "rosconnectioncontroller.h"
#include "maincontroller.h"
#include "libsoromc/constants.h"
#include <ros/ros.h>

#define LogTag "RosConnectionController"

namespace Soro {

RosConnectionController::RosConnectionController(QObject *parent) : QObject(parent)
{
    if (MainController::getSettingsModel()->getIsMaster())
    {
        // We are the master mission control
        setenv("ROS_MASTER_URI", QString("http://localhost:%1").arg(SORO_MC_ROS_MASTER_PORT).toLocal8Bit().constData(), 1);
        initRos();
    }
    else
    {
        MainController::logInfo(LogTag, "Initializing ROS as");
        // We are not the master mission control, we should try to find the ros master
        _udpSocket = new QUdpSocket(this);
        if (!_udpSocket->bind(SORO_MC_BROADCAST_PORT))
        {
            MainController::panic(QString("Cannot bind to mission control UDP broadcast port: %1").arg(_udpSocket->errorString()));
            return;
        }
        connect(_udpSocket, SIGNAL(readyRead()), this, SLOT(udpReadyRead()));
    }
}

void RosConnectionController::udpReadyRead()
{
    while (_udpSocket->hasPendingDatagrams())
    {
        char data[100];
        QHostAddress address;
        quint16 port;
        qint64 len = _udpSocket->readDatagram(data, 100, &address, &port);

        if (strncmp(data, "master", qMax(strlen("master"), (size_t)len)) == 0)
        {
            // Received message from master mission control
            setenv("ROS_MASTER_URI", (QString("http://%1:%2").arg(address.toString(), SORO_MC_ROS_MASTER_PORT)).toLocal8Bit().constData(), 1);
            disconnect(_udpSocket, SIGNAL(readyRead()), this, SLOT(udpReadyRead()));
            delete _udpSocket;
            initRos();
            return;
        }
        else
        {
            MainController::logError(LogTag, "Got invalid message on mission control broadcast port");
        }
    }
}

void RosConnectionController::initRos() {
    int argc = QCoreApplication::arguments().size();
    char *argv[argc];

    for (int i = 0; i < argc; i++) {
        argv[i] = QCoreApplication::arguments()[i].toLocal8Bit().data();
    }

    MainController::logInfo(LogTag, QString("Initializing ROS with master URI \'%1\'").arg(getenv("ROS_MASTER_URI")));
    try
    {
        ros::init(argc, argv, MainController::getMissionControlId().toStdString());
        emit rosInitialized();
    }
    catch(ros::InvalidNameException e)
    {
        MainController::panic(QString("Invalid name exception initializing ROS: %1").arg(e.what()));
        return;
    }
    MainController::logInfo(LogTag, "ROS initialized");
}

} // namepsace Soro
