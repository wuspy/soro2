#ifndef ADDMEDIABOUNCEMESSAGE_H
#define ADDMEDIABOUNCEMESSAGE_H

#include <QByteArray>
#include <QHostAddress>

namespace Soro {

struct AddMediaBounceMessage
{
    AddMediaBounceMessage();
    AddMediaBounceMessage(const QByteArray& payload);
    QByteArray serialize() const;

    QString clientID;
    QHostAddress address;
};

} // namespace Soro

#endif // ADDMEDIABOUNCEMESSAGE_H
