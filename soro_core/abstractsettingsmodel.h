#ifndef ABSTRACTSETTINGSMODEL_H
#define ABSTRACTSETTINGSMODEL_H

#include <QSettings>
#include <QString>
#include <QHash>

namespace Soro {

/* Abstract class that can be made into a settings loader for environment variables.
 */
class AbstractSettingsModel
{
public:
    /* Loads the settings from the environment. This will throw an exception of type QString if an error occurrs
     */
    void load();

protected:
    virtual QHash<QString, int> getKeys() const=0;
    virtual QHash<QString, QVariant> getDefaultValues() const=0;
    QHash<QString, QVariant> _values;
};

} // namespace Soro


#endif // ABSTRACTSETTINGSMODEL_H
