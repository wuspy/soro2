#ifndef QQUICKGSTREAMERSURFACE_H
#define QQUICKGSTREAMERSURFACE_H

#include <QQuickPaintedItem>
#include <QColor>

#include <Qt5GStreamer/QGst/Element>

namespace Soro {

class QQuickGStreamerSurface : public QQuickPaintedItem
{
    Q_OBJECT
    Q_PROPERTY(QColor backgroundColor READ getBackgroundColor WRITE setBackgroundColor)

public:
    QQuickGStreamerSurface();

    void setSink(const QGst::ElementPtr & sink);
    void clearSink();
    void setBackgroundColor(QColor color);
    QColor getBackgroundColor() const;

    void paint(QPainter *painter) override;

private:
    QGst::ElementPtr _sink;
    QColor _backgroundColor;

private:
    void onUpdate();
};

} // namespace Soro

#endif // QQUICKGSTREAMERSURFACE_H
