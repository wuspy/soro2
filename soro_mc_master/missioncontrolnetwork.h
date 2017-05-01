#ifndef MISSIONCONTROLNETWORK_H
#define MISSIONCONTROLNETWORK_H

#include <QObject>
#include <QHostAddress>
#include "soro_core/broadcastreceiver.h"

namespace Soro {

class MissionControlNetwork : public QObject
{
    Q_OBJECT
public:
    explicit MissionControlNetwork(QObject *parent = 0);

Q_SIGNALS:
    void onMissionControlConnected(QString name, QHostAddress address);
    void onMissionControlDisconnected(QString name, QHostAddress address);

private:
    BroadcastReceiver _broadcastReceiver;
    QVector<QString> _mcNames;
    QVector<QHostAddress> _mcAddresses;
};

} // namespace Soro

#endif // MISSIONCONTROLNETWORK_H
