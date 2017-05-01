#ifndef SETTINGSMODEL_H
#define SETTINGSMODEL_H

#include <QSettings>
#include <QString>

#include "soro_core/abstractsettingsmodel.h"

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
        ObserverConfiguration
    };

    SettingsModel::Configuration getConfiguration() const;
    bool getEnableHwRendering() const;
    bool getEnableHwDecoding() const;
    uint getDriveSendInterval() const;

protected:
    QString getFilePath() const;
    QHash<QString, int> getKeys() const;
};

} // namespace Soro

#endif // SETTINGSMODEL_H
