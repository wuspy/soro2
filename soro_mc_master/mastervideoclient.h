#ifndef MASTERVIDEOCLIENT_H
#define MASTERVIDEOCLIENT_H

#include <QObject>
#include <QUdpSocket>
#include <QTimerEvent>

#include <Qt5GStreamer/QGst/Pipeline>
#include <Qt5GStreamer/QGst/Bin>

#include "qmqtt/qmqtt.h"

#include "soro_core/camerasettingsmodel.h"
#include "soro_core/videomessage.h"
#include "settingsmodel.h"

namespace Soro {

class MasterVideoClient : public QObject
{
    Q_OBJECT
public:
    explicit MasterVideoClient(const SettingsModel *settings, const CameraSettingsModel *cameraSettings, QObject *parent = 0);

Q_SIGNALS:
    void bytesDown(quint32 bytes);
    void bounceAddressesChanged(const QHash<QString, QHostAddress>& addresses);

protected:
    void timerEvent(QTimerEvent *e);

private Q_SLOTS:
    void onMqttMessage(const QMQTT::Message &msg);
    void onMqttConnected();
    void onMqttDisconnected();

private:
    char _buffer[65536];
    int _announceTimerId;
    quint16 _nextMqttMsgId;
    QMQTT::Client *_mqtt;
    const SettingsModel *_settings;
    const CameraSettingsModel *_cameraSettings;
    QVector<QUdpSocket*> _videoSockets;
    QVector<QGst::PipelinePtr> _recordPipelines;
    QVector<QGst::BinPtr> _recordBins;
    QUdpSocket *_audioSocket;
    QHash<QString, QHostAddress> _bounceMap;
    QList<QHostAddress> _bounceAddresses;
    QHash<uint, VideoMessage> _videoStateMessages;
};

} // namespace Soro

#endif // MASTERVIDEOCLIENT_H
