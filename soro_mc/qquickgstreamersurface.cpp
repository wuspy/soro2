#include "qquickgstreamersurface.h"
#include <QPainter>
#include <Qt5GStreamer/QGlib/Connect>
#include <Qt5GStreamer/QGlib/Signal>

namespace Soro {

QQuickGStreamerSurface::QQuickGStreamerSurface()
{
    _backgroundColor = QColor("black");
}

void QQuickGStreamerSurface::paint(QPainter *painter)
{
    QRect targetArea(0, 0, width(), height());
    painter->fillRect(targetArea, QBrush(_backgroundColor));
    if (!_sink.isNull()) {
    // This line will show a syntax error if QT_NO_KEYWORDS is not defined, but it will still compile
        QGlib::emit<void>(_sink, "paint", (void *) painter, (qreal)0, (qreal)0, (qreal)width(), (qreal)height());
    }
}

void QQuickGStreamerSurface::setSink(const QGst::ElementPtr &sink)
{
    _sink = sink;
    QGlib::connect(_sink, "update", this, &QQuickGStreamerSurface::onUpdate);
}

void QQuickGStreamerSurface::onUpdate() {
    update();
}

QColor QQuickGStreamerSurface::getBackgroundColor() const {
    return _backgroundColor;
}

void QQuickGStreamerSurface::setBackgroundColor(QColor color) {
    _backgroundColor = color;
}


} // namespace Soro
