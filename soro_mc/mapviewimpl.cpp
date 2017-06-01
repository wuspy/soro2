#include "mapviewimpl.h"
#include "soro_core/logger.h"

#define LogTag "MapViewImpl"

// This function does NOT work if the start point and end point are across the equator or meridian
inline QPointF gpsPointToPixelPoint(Soro::LatLng point, Soro::LatLng startPoint, Soro::LatLng endPoint, qreal pixelWidth, qreal pixelHeight)
{
    return QPointF(pixelWidth * ((point.longitude - startPoint.longitude) / (endPoint.longitude - startPoint.longitude)),
                       (pixelHeight * ((point.latitude - startPoint.latitude) / (endPoint.latitude - startPoint.latitude))));
}

// This function does NOT work if the start point and end point are across the equator or meridian
inline Soro::LatLng pixelPointToGpsPoint(QPointF point, Soro::LatLng startPoint, Soro::LatLng endPoint, qreal pixelWidth, qreal pixelHeight)
{
    return Soro::LatLng((point.x() / pixelWidth) * (endPoint.longitude - startPoint.longitude) + startPoint.longitude,
                        (point.y() / pixelHeight) * (endPoint.latitude - startPoint.latitude) + startPoint.latitude);
}

inline QString degToDms (double deg) {
   double d = floor(deg);
   double minfloat = (deg - d) * 60;
   double m = floor(minfloat);
   double secfloat = (minfloat - m) * 60;
   double s = round(secfloat * 100) / 100.0;

   if (s == 60) {
     m++;
     s = 0;
   }
   if (m == 60) {
     d++;
     m = 0;
   }
   return QString::number((int)d) + "° " + QString::number((int)m) + "' " + QString::number(s, 'f', 2)  + "\"";
}

namespace Soro {

MapViewImpl::MapViewImpl()
{
}

void MapViewImpl::paint(QPainter *painter)
{
    painter->setRenderHint(QPainter::Antialiasing);
    // Draw path
    painter->setPen(QPen(QBrush(QColor("#ff0000")), 2));
    painter->drawImage(QRectF(0, 0, width(), height()), _image, QRectF(0, 0, _image.width(), _image.height()));
    for (int i = 0; i < _locations.size() - 1; ++i)
    {
        painter->drawLine(QPointF(gpsPointToPixelPoint(_locations.value(i), _startCoordinate, _endCoordinate, width(), height())),
                                  QPointF(gpsPointToPixelPoint(_locations.value(i + 1), _startCoordinate, _endCoordinate, width(), height())));
    }
    painter->setPen(QPen(QBrush(QColor("#ffffff")), 2));
    painter->setBrush(QBrush(QColor("#00ff00")));

    // Draw marked positions
    for (QPointF point : _markedPoints)
    {
        painter->drawEllipse(point, 8, 8);
    }

    // Draw current position
    QPointF currentPoint(gpsPointToPixelPoint(_locations.value(_locations.size() - 1), _startCoordinate, _endCoordinate, width(), height()));
    painter->setBrush(QBrush(QColor("#ff0000")));
    /*painter->resetTransform();
    painter->translate(currentPoint);
    painter->rotate(_compassHeading);
    painter->setBrush(QBrush(QColor("#ff0000")));
    painter->drawPie(currentPoint.x() - 25, currentPoint.y() - 25, 50, 50, 1080, 720);*/
    painter->drawEllipse(currentPoint, 8, 8);

    painter->resetTransform();

    // Draw the text of the hover location
    if (_mouseEntered)
    {
        LatLng gpsPointOfMouse = pixelPointToGpsPoint(_mousePosition, _startCoordinate, _endCoordinate, width(), height());
        painter->setBrush(QBrush(QColor("#70000000")));
        QString str = "Lat: " + degToDms(gpsPointOfMouse.latitude) + " (" + QString::number(gpsPointOfMouse.latitude, 'f', 7) + ")\n"
                + "Lng: " + degToDms(gpsPointOfMouse.longitude) + " (" + QString::number(gpsPointOfMouse.longitude, 'f', 7) + ")";
        QRectF bounds = painter->boundingRect(0, 0, width(), height(), Qt::AlignLeft, str);
        bounds.translate(_mousePosition.x() + 25, _mousePosition.y() + 25);
        bounds.setHeight(bounds.height() + 10);
        bounds.setWidth(bounds.width() + 10);
        if (bounds.x() + bounds.width() > width() || bounds.y() + bounds.height() > height())
        {
            bounds.translate(-bounds.width() - 50, -bounds.height() - 50);
        }
        painter->drawRoundedRect(bounds, 6, 6);
        bounds.translate(5, 5);
        painter->drawText(bounds, str);
    }
}

void MapViewImpl::mouseChanged(bool entered, float x, float y)
{
    _mouseEntered = entered;
    _mousePosition.setX(x);
    _mousePosition.setY(y);
    update();
}

void MapViewImpl::markPoint(float x, float y)
{
    for (QPointF point : _markedPoints)
    {
        float diff = qAbs(sqrt(pow(point.x() - x, 2) + pow(point.y() - y, 2)));
        if (diff < 10)
        {
            // Remove point instead of adding it
            _markedPoints.removeAll(point);
            update();
            return;
        }
    }
    _markedPoints.append(QPointF(x, y));
    update();
}

void MapViewImpl::updateLocation(LatLng location)
{
    _locations.append(location);
    update();
}

void MapViewImpl::updateHeading(double heading)
{
    _compassHeading = heading;
    update();
}

void MapViewImpl::setStartCoordinate(LatLng location)
{
    _startCoordinate = location;
}

void MapViewImpl::setEndCoordinate(LatLng location)
{
    _endCoordinate = location;
}

QString MapViewImpl::getImage() const
{
    return _imagePath;
}

void MapViewImpl::setImage(QString image)
{
    if (_imagePath != image)
    {
        _imagePath = image;
        _image.load(_imagePath);
        setWidth(_image.width());
        setHeight(_image.height());
    }
}

} // namespace Soro
