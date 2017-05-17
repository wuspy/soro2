#include "videomessage.h"

#include <QDataStream>

namespace Soro {

VideoMessage::VideoMessage()
{
    camera_index = 0;
    camera_offset = 0;
    camera_computerIndex = 0;
    isStereo = false;
    camera_offset2 = 0;
}

VideoMessage::VideoMessage(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::BigEndian);
    QString profileStr;

    stream >> profileStr;
    stream >> camera_computerIndex;
    stream >> camera_index;
    stream >> camera_name;
    stream >> camera_offset;
    stream >> camera_productId;
    stream >> camera_serial;
    stream >> camera_vendorId;
    stream >> isStereo;
    stream >> camera_offset2;
    stream >> camera_productId2;
    stream >> camera_serial2;
    stream >> camera_vendorId2;

    profile = GStreamerUtil::VideoProfile(profileStr);
}

VideoMessage::VideoMessage(quint16 cameraIndex, const CameraSettingsModel::Camera &cam)
{
    camera_computerIndex = cam.computerIndex;
    camera_index = cameraIndex;
    camera_name = cam.name;
    camera_offset = cam.offset;
    camera_productId = cam.productId;
    camera_serial = cam.serial;
    camera_vendorId = cam.vendorId;
    isStereo = cam.isStereo;
    camera_offset2 = cam.offset2;
    camera_productId2 = cam.productId2;
    camera_serial2 = cam.serial2;
    camera_vendorId2 = cam.vendorId2;
}

VideoMessage::operator QByteArray() const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << profile.toString()
           << camera_computerIndex
           << camera_index
           << camera_name
           << camera_offset
           << camera_productId
           << camera_serial
           << camera_vendorId
           << isStereo
           << camera_offset2
           << camera_productId2
           << camera_serial2
           << camera_vendorId2;

    return payload;
}

} // namespace Soro
