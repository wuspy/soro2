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
        int id;
        QString name;
        QString serial;
        QString vendorId;
        QString productId;
    };

    /* Loads the camera definitions from the camera settings file. If unsuccessful,
     * this will return false and set the class error string, which can be accessed with errorString()
     */
    void load();

    /* Returns a list of the camera definitions loaded from the settings file,
     * or an empty list if the file has not yet been loaded or if an error
     * occurred when loading the file
     */
    const QList<CameraSettingsModel::Camera> getCameras() const;
    int getCameraIndexById(int id) const;
    int getCameraCount() const;

private:
    QMap<int, CameraSettingsModel::Camera> _cameras;
};

} // namespace Soro

#endif // CAMERASETTINGSMODEL_H
