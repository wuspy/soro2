#ifndef QQUICKGSTREAMERSURFACE_H
#define QQUICKGSTREAMERSURFACE_H

#include <QQuickPaintedItem>
#include <QColor>

#include <Qt5GStreamer/QGst/Element>

namespace Soro {

class QQuickGStreamerSurface : public QQuickPaintedItem
{
    Q_OBJECT
    Q_PROPERTY(bool EnableHardwareRendering READ getEnableHardwareRendering WRITE setEnableHardwareRendering)

public:
    QQuickGStreamerSurface();
    ~QQuickGStreamerSurface();

    void paint(QPainter *painter) override;

    QGst::ElementPtr videoSink();

    bool getEnableHardwareRendering() const;
    void setEnableHardwareRendering(bool enableHardwareRendering);

private:
    QGst::ElementPtr _sink;
    bool _useHardwareRendering;

private:
    void onUpdate();
};

} // namespace Soro

#endif // QQUICKGSTREAMERSURFACE_H
