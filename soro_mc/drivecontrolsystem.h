#ifndef DRIVECONTROLSYSTEM_H
#define DRIVECONTROLSYSTEM_H

#include <QObject>
#include "gamepadcontroller.h"
#include "connectionstatuscontroller.h"
#include "ros_generated/drive.h"
#include "ros/ros.h"
#include <QTimerEvent>

namespace Soro {

class DriveControlSystem : public QObject
{
    Q_OBJECT

public:
    explicit DriveControlSystem(int sendInterval, const GamepadController *gamepad,
                                ConnectionStatusController *connectionStatusController,
                                QObject *parent = 0);

    void setSkidSteerFactor(float factor);
    float getSkidSteerFactor() const;

    void setLimit(float limit);
    float getLimit() const;

private:
    ros_generated::drive buildDriveMessage();

    QString _gamepadName;
    const GamepadController *_gamepad;
    ConnectionStatusController *_connectionStatusController;

    ros::NodeHandle _nh;
    ros::Publisher _drivePublisher;

    int _sendTimerId;
    float _skidSteerFactor;
    float _limit;

protected:
    void timerEvent(QTimerEvent* e);

};


}

#endif // DRIVECONTROLSYSTEM_H
