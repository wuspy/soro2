#include "compassmessage.h"

#include <QDataStream>

namespace Soro {

CompassMessage::CompassMessage() { }

CompassMessage::CompassMessage(const QByteArray &payload)
{
    heading = QString(payload).toDouble();
}

CompassMessage::operator QByteArray() const
{
    return QByteArray(QString::number(heading, 'f', 4).toLatin1().constData());
}

} // namespace Soro
