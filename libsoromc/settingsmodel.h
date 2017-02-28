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
    bool load();

    /* Writes the current settings back to the file. If unsuccessful, this will return false
     * and set the class error string, which can be accessed with errorString()
     */
    bool write();

    // Getters/Setters

    bool getIsMaster() const;
    void setIsMaster(bool master);
    QString errorString() const;

private:
    /* A helper function for load() that both checks if a key exists in the file,
     * and sets the error string if it doesn't
     */
    bool checkKeyExists(QString key, QString *error);
    QString _errorString;
    QSettings *_settings = nullptr;
};

} // namespace Soro

#endif // SETTINGSMODEL_H
