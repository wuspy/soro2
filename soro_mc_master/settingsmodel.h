#ifndef SETTINGSMODEL_H
#define SETTINGSMODEL_H

#include <QSettings>
#include <QString>
#include <QHostAddress>

#include "soro_core/abstractsettingsmodel.h"

namespace Soro {

/* Main settings loader class for the master application
 */
class SettingsModel: public AbstractSettingsModel
{
public:
    uint getPingInterval() const;
    uint getDataRateCalcInterval() const;
    QHostAddress getMqttBrokerAddress() const;

protected:
    QHash<QString, int> getKeys() const override;
    QHash<QString, QVariant> getDefaultValues() const override;
};

} // namespace Soro

#endif // SETTINGSMODEL_H
