#ifndef CAMERASETTINGSMODEL_H
#define CAMERASETTINGSMODEL_H

#include <QString>
#include <QMap>
#include <QList>

namespace Soro {

class CameraSettingsModel
{
public:
    /* This struct represents a single camera definition
     */
    struct Camera {
        QString name;
        QString serial;
        QString vendorId;
        QString productId;
    };

    /* Loads the camera definitions from the camera settings file. If unsuccessful,
     * this will return false and set the class error string, which can be accessed with errorString()
     */
    void load();

    Camera getCamera(uint index) const;
    int getCameraCount() const;

private:
    QList<CameraSettingsModel::Camera> _cameras;
};

} // namespace Soro

#endif // CAMERASETTINGSMODEL_H
