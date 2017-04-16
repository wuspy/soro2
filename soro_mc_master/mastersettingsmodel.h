#ifndef SETTINGSMODEL_H
#define SETTINGSMODEL_H

#include <QSettings>
#include <QString>

namespace Soro {

/* Main settings loader class for the master application
 */
class MasterSettingsModel
{
public:
    MasterSettingsModel();
    ~MasterSettingsModel();

    /* Loads the main settings file. If unsuccessful, this will throw a QString containing an error message
     */
    void load();

    uint getPingInterval() const;
    uint getBitrateInterval() const;

private:
    QSettings *_settings = nullptr;
};

} // namespace Soro

#endif // SETTINGSMODEL_H
