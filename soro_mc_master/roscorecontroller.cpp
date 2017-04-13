#include "roscorecontroller.h"
#include "libsoromc/constants.h"

#include <QDebug>

RosCoreController::RosCoreController(QObject *parent) : QObject(parent)
{
    qDebug() << "Starting";
    _roscoreProcess.setProcessChannelMode(QProcess::ForwardedChannels);
    _roscoreProcess.execute("/bin/bash", QStringList() << "roscore");
    qDebug() << "Finished";
}
