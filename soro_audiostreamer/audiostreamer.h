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
class AudioStreamer : public QObject {
    Q_OBJECT
public:
    AudioStreamer(QObject *parent = 0);
    ~AudioStreamer();

public Q_SLOTS:
    void stop();
    void stream(const QString& address, int port, int bindPort, const QString& profile);
    void heartbeat();

protected:
    void timerEvent(QTimerEvent *e);

private:
    QGst::PipelinePtr createPipeline();

    void stopPrivate(bool sendReady);

    int _watchdogTimerId;
    QGst::PipelinePtr _pipeline;
    QDBusInterface *_parentInterface;

private Q_SLOTS:
    void onBusMessage(const QGst::MessagePtr & message);
};

} // namespace Soro

#endif // SORO_GST_MEDIASTREAMER_H
