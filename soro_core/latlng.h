#ifndef LATLNG_H
#define LATLNG_H

#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT LatLng
{
    double latitude;
    double longitude;

    LatLng()
    {
        latitude = 0.0;
        longitude = 0.0;
    }

    LatLng(double latitude, double longitude)
    {
        this->latitude = latitude;
        this->longitude = longitude;
    }
};

} // namespace Soro

#endif // LATLNG_H
