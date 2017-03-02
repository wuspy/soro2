#ifndef DRIVECONTROLSYSTEM_H
#define DRIVECONTROLSYSTEM_H

#include <QObject>
#include <gamepadcontroller.h>
#include "libsoromc/drivemessage.h"
#include "ros/ros.h"
#include <QTimerEvent>

// ***TODO***
// ROS hangs the application if connection to master is not immediately successful
// All functionality associated with ROS has been commented out until this can be resolved
namespace Soro {

class DriveControlSystem : public QObject
{
    Q_OBJECT

public:
    explicit DriveControlSystem(QObject *parent = 0);

private:
    Soro::Messages::drive buildDriveMessage();

    QString _gamepadName;
    //ros::NodeHandle driveHandle;
    //ros::Publisher drivePublisher;

    int sendTimerId;

protected:
    void timerEvent(QTimerEvent* e);

};


}

#endif // DRIVECONTROLSYSTEM_H
