#ifndef ABSTRACTSETTINGSMODEL_H
#define ABSTRACTSETTINGSMODEL_H

#include <QSettings>
#include <QString>
#include <QHash>

namespace Soro {

/* Main settings class for the application, handles loading settings
 */
class AbstractSettingsModel
{
public:
    ~AbstractSettingsModel();

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
