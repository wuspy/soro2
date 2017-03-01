#include "qquickgstreamersurface.h"
#include <QPainter>
#include <Qt5GStreamer/QGlib/Connect>
#include <Qt5GStreamer/QGlib/Signal>

namespace Soro {

QQuickGStreamerSurface::QQuickGStreamerSurface()
{

}

void QQuickGStreamerSurface::paint(QPainter *painter)
{
    QRect targetArea(0, 0, width(), height());
    // This line will show a syntax error if QT_NO_KEYWORDS is not defined, but it will still compile
    QGlib::emit<void>(_sink, "paint", (void *) painter, (qreal)0, (qreal)0, (qreal)width(), (qreal)height());
}

void QQuickGStreamerSurface::setSink(const QGst::ElementPtr &sink)
{
    clearSink();
    QGlib::connect(sink, "update", this, &QQuickGStreamerSurface::onUpdate);
    _sink = QGst::ElementPtr(sink);
}

void QQuickGStreamerSurface::clearSink() {
    if (!_sink.isNull()) {
        QGlib::disconnect(_sink, "update", this, &QQuickGStreamerSurface::onUpdate);
        _sink.clear();
    }
}

void QQuickGStreamerSurface::onUpdate() {
    update();
}

} // namespace Soro
