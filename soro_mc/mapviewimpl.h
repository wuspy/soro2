#ifndef MAPVIEWIMPL_H
#define MAPVIEWIMPL_H

#include <QQuickPaintedItem>

namespace Soro {

class MapViewImpl : public QQuickPaintedItem
{
    Q_OBJECT
public:
    MapViewImpl();
    void paint(QPainter *painter);

    struct Location {
        double lat;
        double lng;
    };

private:
    QList<Location> _locations;
    double _compassHeading;
};

} // namespace Soro

#endif // MAPVIEWIMPL_H
