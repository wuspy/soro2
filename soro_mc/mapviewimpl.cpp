#include "mapviewimpl.h"
#include "soro_core/logger.h"

#define LogTag "MapViewImpl"

namespace Soro {

MapViewImpl::MapViewImpl()
{
}

void MapViewImpl::paint(QPainter *painter)
{
    painter->drawImage(QRectF(0, 0, width(), height()), _image, QRectF(0, 0, _image.width(), _image.height()));
    for (int i = 0; i < _locations.size() - 1; ++i)
    {
        //painter->drawLine(QPointF());
    }
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
