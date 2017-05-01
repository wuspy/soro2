#ifndef MASTERLOCATOR_H
#define MASTERLOCATOR_H

#include <QObject>
#include <QUdpSocket>
#include "soro_core/constants.h"

namespace Soro {

class MasterLocator : public QObject
{
    Q_OBJECT
public:
    explicit MasterLocator(QObject *parent = 0);

Q_SIGNALS:
    void masterFound(QHostAddress address);

private:
    QUdpSocket _socket;
};

} // namespace Soro

#endif // MASTERLOCATOR_H
