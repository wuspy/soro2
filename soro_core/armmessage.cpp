#include "armmessage.h"

namespace Soro {

ArmMessage::ArmMessage() { }

ArmMessage::ArmMessage(const QByteArray &payload)
{
    masterArmData = payload;
}

ArmMessage::operator QByteArray() const
{
    return masterArmData;
}

} // namespace Soro
