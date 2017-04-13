#ifndef SETTINGSMODEL_H
#define SETTINGSMODEL_H

#include <QSettings>
#include <QString>

namespace Soro {

/* Main settings class for the application, handles loading settings
 */
class SettingsModel
{
public:
    SettingsModel();
    ~SettingsModel();

    enum Configuration {
        DriverConfiguration,
        ArmOperatorConfiguration,
        CameraOperatorConfiguration,
        ObserverConfiguration
    };

    /* Loads the main settings file. If unsuccessful, this will throw a QString containing an error message
     */
    void load();

    SettingsModel::Configuration getConfiguration() const;

private:
    QSettings *_settings = nullptr;
};

} // namespace Soro

#endif // SETTINGSMODEL_H
