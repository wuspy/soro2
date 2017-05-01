#ifndef MASTERVIDEOCONTROLLER_H
#define MASTERVIDEOCONTROLLER_H

#include <QObject>
#include <QUdpSocket>
#include <QTimerEvent>

#include "soro_core/camerasettingsmodel.h"
#include "soro_core/broadcastreceiver.h"

namespace Soro {

class MasterVideoController : public QObject
{
    Q_OBJECT
public:
    explicit MasterVideoController(const CameraSettingsModel *cameraSettings, QHostAddress serverAddress, QObject *parent = 0);

Q_SIGNALS:
    void bitsDown(quint32 bits);

protected:
    void timerEvent(QTimerEvent *e);

private:
    void onSocketReadyRead(QUdpSocket *socket, quint16 bouncePort);
    void addPunchHost(QHostAddress address);
    void removePunchHost(QHostAddress address);

    char _buffer[65536];

    int _punchTimerId;
    QHostAddress _serverAddress;
    const CameraSettingsModel *_cameraSettings;
    QVector<QUdpSocket*> _videoSockets;
    QUdpSocket *_audioSocket;
    QVector<QHostAddress> _bounceAddresses;
};

} // namespace Soro

#endif // MASTERVIDEOCONTROLLER_H
