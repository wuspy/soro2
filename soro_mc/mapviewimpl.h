#ifndef MAPVIEWIMPL_H
#define MAPVIEWIMPL_H

#include <QQuickPaintedItem>
#include <QPainter>
#include <QImage>

#include "soro_core/latlng.h"

namespace Soro {

class MapViewImpl : public QQuickPaintedItem
{
    Q_OBJECT
    Q_PROPERTY(QString image READ getImage WRITE setImage)

public:
    MapViewImpl();
    void paint(QPainter *painter);

    QString getImage() const;
    void setImage(QString image);

public Q_SLOTS:
    void updateLocation(LatLng location);
    void updateHeading(double heading);

private:
    QList<LatLng> _locations;
    double _compassHeading;
    QImage _image;
    QString _imagePath;
};

} // namespace Soro

#endif // MAPVIEWIMPL_H
