#ifndef GSTREAMERPIPELINEWATCH_H
#define GSTREAMERPIPELINEWATCH_H

#include <QObject>
#include <Qt5GStreamer/QGst/Pipeline>
#include <Qt5GStreamer/QGst/Message>

namespace Soro {

/* This class is designed to listen to bus messages on GStreamer pipelines, and emit these
 * messages in a Qt-like syntax. It also accepts an ID, which is also emitted with all
 * class signals, to make it easy to listen to and identify many pipelines at once.
 */
class GStreamerPipelineWatch : public QObject
{
    Q_OBJECT
public:
    explicit GStreamerPipelineWatch(int id, QGst::PipelinePtr pipeline, QObject *parent = 0);

Q_SIGNALS:
    void error(QString message, int id);

private Q_SLOTS:
    void onBusMessage(const QGst::MessagePtr & message);

private:
    int _id;
};

} // namespace Soro

#endif // GSTREAMERPIPELINEWATCH_H
