#ifndef SORO_GST_MEDIASTREAMER_H
#define SORO_GST_MEDIASTREAMER_H

#include <QObject>
#include <QCoreApplication>
#include <QtDBus>
#include <QTimerEvent>

#include <Qt5GStreamer/QGst/Pipeline>
#include <Qt5GStreamer/QGlib/RefPointer>
#include <Qt5GStreamer/QGst/Message>

namespace Soro {

/*
 * Uses a gstreamer backend to stream media to a remote address. This class does not run in the main process,
 * instead it runs in a child process is controlled by a corresponding MediaServer in the main process.
 */
class VideoStreamer : public QObject {
    Q_OBJECT
public:
    VideoStreamer(QString streamName, QObject *parent = 0);
    ~VideoStreamer();

public Q_SLOTS:
    void stop();
    void stream(const QString &device, const QString &address, int port, const QString &profile, bool vaapi);
    void streamStereo(const QString &leftDevice, const QString &rightDevice, const QString &address, int port, const QString &profile, bool vaapi);
    void heartbeat();

protected:
    void timerEvent(QTimerEvent *e);

private:
    QGst::PipelinePtr createPipeline();

    void stopPrivate(bool sendReady);

    int _watchdogTimerId;
    QString _name;
    QGst::PipelinePtr _pipeline;
    QDBusInterface *_parentInterface;

private Q_SLOTS:
    void onBusMessage(const QGst::MessagePtr & message);
};

} // namespace Soro

#endif // SORO_GST_MEDIASTREAMER_H
