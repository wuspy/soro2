#ifndef DRIVECONTROLSYSTEM_H
#define DRIVECONTROLSYSTEM_H

#include <QObject>
#include <gamepadcontroller.h>
#include "libsoromc/drivemessage.h"
#include "ros/ros.h"
#include <SDL2/SDL.h>
#include <QTimerEvent>


namespace Soro {

class DriveControlSystem : public QObject
{
    Q_OBJECT

public:
    explicit DriveControlSystem(QObject *parent = 0);

    Soro::Messages::drive* buildDriveMessage(float x, float y);
    void setDriveMode(int i);
    int getDriveMode();
    void setDriveEnabled(bool e);
    bool getDriveEnabled();

private:

    int _sendTimerId;
    int _mode;          //mode: 1 for single stick, 2 for dual
    bool _enabled;       //is drivecontrol enabled

    ros::NodeHandle _driveHandle;
    ros::Publisher _drivePublisher;

protected:
    void timerEvent(QTimerEvent* e);

};


}

#endif // DRIVECONTROLSYSTEM_H
