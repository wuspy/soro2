#ifndef VIDEOMESSAGE_H
#define VIDEOMESSAGE_H

#include <QByteArray>

#include "abstractmessage.h"
#include "gstreamerutil.h"
#include "camerasettingsmodel.h"
#include "soro_core_global.h"

namespace Soro {

struct SORO_CORE_EXPORT VideoMessage : public AbstractMessage
{
    VideoMessage();
    VideoMessage(const QByteArray& payload);
    VideoMessage(quint16 cameraindex, const CameraSettingsModel::Camera& cam);
    operator QByteArray() const override;

    GStreamerUtil::VideoProfile profile;
    quint8 camera_computerIndex;
    QString camera_name;
    quint16 camera_index;
    bool isStereo;

    QString camera_serial;
    QString camera_vendorId;
    QString camera_productId;
    quint8 camera_offset;

    QString camera_serial2;
    QString camera_vendorId2;
    QString camera_productId2;
    quint8 camera_offset2;

};

} // namespace Soro

#endif // VIDEOMESSAGE_H
