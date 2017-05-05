#ifndef MASTERVIDEOCONTROLLER_H
#define MASTERVIDEOCONTROLLER_H

#include <QObject>
#include <QUdpSocket>
#include <QTimerEvent>

#include "soro_core/camerasettingsmodel.h"
#include "soro_core/rosnodelist.h"

namespace Soro {

class MasterVideoController : public QObject
{
    Q_OBJECT
public:
    explicit MasterVideoController(const CameraSettingsModel *cameraSettings, const RosNodeList *rosNodeList, QObject *parent = 0);

Q_SIGNALS:
    void bytesDown(quint32 bytes);

private Q_SLOTS:
    void onRosNodeListUpdated();

private:
    void onSocketReadyRead(QUdpSocket *socket, quint16 bouncePort);

    char _buffer[65536];
    int _announceTimerId;
    const RosNodeList *_rosNodeList;
    const CameraSettingsModel *_cameraSettings;
    QVector<QUdpSocket*> _videoSockets;
    QUdpSocket *_audioSocket;
    QVector<QHostAddress> _bounceAddresses;
};

} // namespace Soro

#endif // MASTERVIDEOCONTROLLER_H
