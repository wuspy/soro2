#ifndef ARMCONTROLSYSTEM_H
#define ARMCONTROLSYSTEM_H

#include <QObject>
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"

namespace Soro {

class ArmControlSystem: public QObject
{
    Q_OBJECT

public:
    explicit ArmControlSystem(QObject *parent=0);

private:
    ros::NodeHandle _nh;
    ros::Publisher _armPublisher;

};

}

#endif // ARMCONTROLSYSTEM_H
