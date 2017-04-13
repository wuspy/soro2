#ifndef DRIVECONTROLSYSTEM_H
#define DRIVECONTROLSYSTEM_H

#include <QObject>
#include <gamepadcontroller.h>
#include "libsoromc/drivemessage.h"
#include "ros/ros.h"
#include <QTimerEvent>

namespace Soro {

class DriveControlSystem : public QObject
{
    Q_OBJECT

public:
    explicit DriveControlSystem(QObject *parent = 0);

private:
    message_gen::drive buildDriveMessage();

    QString _gamepadName;
    ros::Publisher _drivePublisher;

    int _sendTimerId;

protected:
    void timerEvent(QTimerEvent* e);

};


}

#endif // DRIVECONTROLSYSTEM_H
