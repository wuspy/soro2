#ifndef SETTINGSMODEL_H
#define SETTINGSMODEL_H

#include <QSettings>
#include <QString>

namespace Soro {

/* Main settings class for the application, handles loading settings and (if needed)
 * writing back to the settings file.
 */
class SettingsModel
{
public:
    SettingsModel();
    ~SettingsModel();

    /* Loads the main settings file. If unsuccessful, this will return false and
     * set the class error string, which can be accessed with errorString()
     */
    void load();

    bool getIsMaster() const;

private:
    QSettings *_settings = nullptr;
};

} // namespace Soro

#endif // SETTINGSMODEL_H
