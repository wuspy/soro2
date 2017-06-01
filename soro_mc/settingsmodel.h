#ifndef SETTINGSMODEL_H
#define SETTINGSMODEL_H

#include <QSettings>
#include <QString>
#include <QHostAddress>

#include "soro_core/abstractsettingsmodel.h"
#include "soro_core/latlng.h"

namespace Soro {

/* Main settings class for the soro_mc application
 */
class SettingsModel: public AbstractSettingsModel
{
public:
    enum Configuration {
        DriverConfiguration,
        ArmOperatorConfiguration,
        CameraOperatorConfiguration,
        ScienceArmOperatorConfiguration,
        ScienceCameraOperatorConfiguration,
        ObserverConfiguration
    };

    enum DriveInputMode
    {
        DriveInputMode_TwoStick,
        DriveInputMode_SingleStick
    };

    enum CameraGimbalInputMode
    {
        CameraGimbalInputMode_LeftStick,
        CameraGimbalInputMode_LeftStickYInverted,
        CameraGimbalInputMode_RightStick,
        CameraGimbalInputMode_RightStickYInverted,
        CameraGimbalInputMode_BillsWay,
    };

    SettingsModel::Configuration getConfiguration() const;
    bool getEnableHwRendering() const;
    bool getEnableHwDecoding() const;
    uint getDriveSendInterval() const;
    DriveInputMode getDriveInputMode() const;
    CameraGimbalInputMode getCameraGimbalInputMode() const;
    float getDriveSkidSteerFactor() const;
    float getDrivePowerLimit() const;
    uint getCameraGimbalSendInterval() const;
    QHostAddress getMqttBrokerAddress() const;
    QString getMapImage() const;
    LatLng getMapStartCoordinates() const;
    LatLng getMapEndCoordinates() const;

protected:
    QHash<QString, int> getKeys() const override;
    QHash<QString, QVariant> getDefaultValues() const override;
};

} // namespace Soro

#endif // SETTINGSMODEL_H
