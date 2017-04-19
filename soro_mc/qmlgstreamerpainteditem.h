#ifndef QMLGSTREAMERPAINTEDITEM_H
#define QMLGSTREAMERPAINTEDITEM_H

#include <QQuickPaintedItem>
#include <Qt5GStreamer/QGst/Element>

namespace Soro {

class QmlGStreamerPaintedItem : public QQuickPaintedItem
{
    Q_OBJECT

public:
    QmlGStreamerPaintedItem(QQuickItem *parent=0);
    ~QmlGStreamerPaintedItem();

    void paint(QPainter *painter) override;

    QGst::ElementPtr videoSink();

protected:
    void onUpdate();

private:
    QGst::ElementPtr _sink;
};

} // namespace Soro

#endif // QMLGSTREAMERPAINTEDITEM_H
