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
    void setStartCoordinate(LatLng location);
    void setEndCoordinate(LatLng location);
    Q_INVOKABLE void markPoint(float x, float y);
    Q_INVOKABLE void mouseChanged(bool entered, float x, float y);

private:
    QList<LatLng> _locations;
    double _compassHeading;
    QImage _image;
    LatLng _startCoordinate;
    LatLng _endCoordinate;
    QString _imagePath;
    QList<LatLng> _markedPointsCoordinates;
    QList<QPointF> _markedPoints;
    bool _mouseEntered;
    QPointF _mousePosition;
};

} // namespace Soro

#endif // MAPVIEWIMPL_H
