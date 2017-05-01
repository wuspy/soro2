#ifndef SETTINGSMODEL_H
#define SETTINGSMODEL_H

#include <QSettings>
#include <QString>

#include "soro_core/abstractsettingsmodel.h"

namespace Soro {

/* Main settings loader class for the master application
 */
class SettingsModel: public AbstractSettingsModel
{
public:
    uint getPingInterval() const;
    uint getBitrateInterval() const;

protected:
    QString getFilePath() const override;
    QHash<QString, int> getKeys() const override;
};

} // namespace Soro

#endif // SETTINGSMODEL_H
