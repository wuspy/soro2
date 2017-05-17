#ifndef SETTINGSMODEL_H
#define SETTINGSMODEL_H

#include <QSettings>
#include <QString>
#include <QHostAddress>

#include "soro_core/abstractsettingsmodel.h"

namespace Soro {

/* Main settings loader class for the video server application
 */
class SettingsModel: public AbstractSettingsModel
{
public:
    QHostAddress getMqttBrokerAddress() const;

protected:
    QHash<QString, int> getKeys() const override;
    QHash<QString, QVariant> getDefaultValues() const override;
};

} // namespace Soro

#endif // SETTINGSMODEL_H
