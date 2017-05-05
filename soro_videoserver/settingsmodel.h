#ifndef SETTINGSMODEL_H
#define SETTINGSMODEL_H

#include <QSettings>
#include <QString>

#include "soro_core/abstractsettingsmodel.h"

namespace Soro {

/* Main settings loader class for the video server application
 */
class SettingsModel: public AbstractSettingsModel
{
public:
    uint getComputerIndex() const;
    bool getUseH264Vaapi() const;
    bool getUseVP8Vaapi() const;
    bool getUseMpeg2Vaapi() const;
    bool getUseH265Vaapi() const;
    bool getUseJpegVaapi() const;

protected:
    QHash<QString, int> getKeys() const override;
    QHash<QString, QVariant> getDefaultValues() const override;
};

} // namespace Soro

#endif // SETTINGSMODEL_H
