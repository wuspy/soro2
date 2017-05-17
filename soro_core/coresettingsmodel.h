#ifndef CORESETTINGSMODEL_H
#define CORESETTINGSMODEL_H

#include <QSettings>
#include <QString>
#include <QHostAddress>

#include "abstractsettingsmodel.h"
#include "soro_core_global.h"

namespace Soro {

/* Settings model that will load settings that should be needed by
 * any application wanting to use this library
 */
class SORO_CORE_EXPORT CoreSettingsModel: public AbstractSettingsModel
{
public:
    QHostAddress getMqttBrokerAddress() const;
    QHostAddress getMediaServerAddress() const;
    quint16 getGimbalSystemPort() const;
    quint16 getDriveSystemPort() const;
    quint16 getArmSystemPort() const;
    quint16 getMqttBrokerPort() const;
    quint16 getServerAudioPort() const;
    quint16 getServerFirstVideoPort() const;
    uint getMaxCameras() const;
    QString getSettingsDirPath() const;

    bool getUseH264VaapiEncode() const;
    bool getUseVP8VaapiEncode() const;
    bool getUseMpeg2VaapiEncode() const;
    bool getUseH265VaapiEncode() const;
    bool getUseJpegVaapiEncode() const;
    bool getUseH264VaapiDecode() const;
    bool getUseVP8VaapiDecode() const;
    bool getUseMpeg2VaapiDecode() const;
    bool getUseH265VaapiDecode() const;
    bool getUseJpegVaapiDecode() const;

protected:
    QHash<QString, int> getKeys() const override;
    QHash<QString, QVariant> getDefaultValues() const override;
};

} // namespace Soro

#endif // CORESETTINGSMODEL_H
