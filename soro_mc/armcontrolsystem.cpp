#include "armcontrolsystem.h"
#include "maincontroller.h"
#include "libsoromc/logger.h"
#include "libsoromc/constants.h"

#define LogTag "ArmControlSystem"

namespace Soro
{

ArmControlSystem::ArmControlSystem(QObject *parent) : QObject(parent)
{
    Logger::logInfo(LogTag, "Creating publisher...");
    _armPublisher = _nh.advertise<std_msgs::UInt8MultiArray>("arm", 1);
    if (!_armPublisher) MainController::panic(LogTag, "Failed to create ROS publisher for arm topic");
    Logger::logInfo(LogTag, "Arm Publisher created");
}

void ArmControlSystem::initArmUdpSocket(){
    Logger::logInfo(LogTag, "Initializing arm udp socket...");

    try{
        if(!_armUdpSocket.bind(SORO_NET_MASTER_ARM_PORT)){
            MainController::panic(LogTag, QString("Cannot bind to mission control arm port: %1").arg(_armUdpSocket.errorString()));
                        return;
        }
        connect(&_armUdpSocket, &QUdpSocket::readyRead, this, &ArmControlSystem::readArmData);
    }
    catch(QString err){
        MainController::panic(LogTag, QString("Error initializing arm udp socket: %1").arg(err));
                return;
    }
}

void ArmControlSystem::readArmData()
{
    Logger::logInfo(LogTag, "Received arm datagram");
    QByteArray armData;
    QHostAddress address;
    quint16 port;

    while (_armUdpSocket.hasPendingDatagrams())
    {
        //resize buffer and fill with 0's
        armData.fill(0, _armUdpSocket.pendingDatagramSize());

        _armUdpSocket.readDatagram(armData.data(), armData.size(), &address, &port);

        std_msgs::UInt8MultiArray armMessage;
        armMessage.data.clear();

        for(int i = 0; i < armData.size(); i++){
            armMessage.data.push_back(armData.at(i));
        }

        _armPublisher.publish(armMessage);

        Logger::logInfo(LogTag, QString("Published arm message containing: %1").arg(QString(armData.data())));



    }
}

}
