#ifndef MEDIABOUNCER_H
#define MEDIABOUNCER_H

#include "udpbouncer.h"

#include <QObject>

namespace Soro {

/* Forwards audio and video to all mission controls
 */
class MediaBouncer : public QObject
{
    Q_OBJECT
public:
    explicit MediaBouncer(QObject *parent = 0);
    ~MediaBouncer();

Q_SIGNALS:
    void bitsDown(int bits);

public Q_SLOTS:
    void addBounceAddress(QHostAddress address);
    void removeBounceAddress(QHostAddress address);

private:
    QVector<UdpBouncer*> _bouncers;
};

} // namespace Soro

#endif // MEDIABOUNCER_H
