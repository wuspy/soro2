#ifndef SETTINGSMODEL_H
#define SETTINGSMODEL_H

#include <QSettings>
#include <QString>

#include "libsoromc/abstractsettingsmodel.h"

namespace Soro {

/* Main settings class for the application, handles loading settings
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