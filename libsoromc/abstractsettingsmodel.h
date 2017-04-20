#ifndef ABSTRACTSETTINGSMODEL_H
#define ABSTRACTSETTINGSMODEL_H

#include <QSettings>
#include <QString>
#include <QHash>

namespace Soro {

/* Abstract class that can be made into a settings loader for 'key=value' formatted
 * configuration files. It is based on QSettings, so all limitations and requirements
 * of QSettings applies to this class as well.
 */
class AbstractSettingsModel
{
public:
    ~AbstractSettingsModel();

    /* Loads the settings from file. This will throw an exception of type QString if an error occurrs
     */
    void load();

protected:
    QSettings *_settings = nullptr;

    virtual QString getFilePath() const=0;
    virtual QHash<QString, int> getKeys() const=0;

private:
    void fatalLoadError(QString message);
};

} // namespace Soro


#endif // ABSTRACTSETTINGSMODEL_H
