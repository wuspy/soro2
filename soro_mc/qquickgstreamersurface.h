#ifndef QQUICKGSTREAMERSURFACE_H
#define QQUICKGSTREAMERSURFACE_H

#include <QQuickPaintedItem>

#include <Qt5GStreamer/QGst/Element>

namespace Soro {

class QQuickGStreamerSurface : public QQuickPaintedItem
{
    Q_OBJECT
public:
    QQuickGStreamerSurface();

    void setSink(const QGst::ElementPtr & sink);
    void clearSink();

    void paint(QPainter *painter) override;

private:
    QGst::ElementPtr _sink;

private:
    void onUpdate();
};

} // namespace Soro

#endif // QQUICKGSTREAMERSURFACE_H
