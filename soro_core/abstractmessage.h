#ifndef ABSTRACTMESSAGE_H
#define ABSTRACTMESSAGE_H

#include <QByteArray>

#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT AbstractMessage
{
    virtual operator QByteArray() const=0;
};

} // namespace Soro

#endif // ABSTRACTMESSAGE_H
