#include "mapviewimpl.h"
#include "soro_core/logger.h"

#define LogTag "MapViewImpl"

namespace Soro {

MapViewImpl::MapViewImpl()
{
    if (!_mapImage.load(QCoreApplication::applicationDirPath() + "/../maps/current"))
    {
        LOG_E(LogTag, "Unable to load map asset");
    }
    setWidth(_mapImage.width());
    setHeight(_mapImage.height());
}

void MapViewImpl::paint(QPainter *painter)
{
    painter->drawImage(0, 0, _mapImage);
}

} // namespace Soro
