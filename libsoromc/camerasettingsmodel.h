#ifndef CAMERASETTINGSMODEL_H
#define CAMERASETTINGSMODEL_H

#include <QString>
#include <QVector>

#include "soroexception.h"

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

    /* Loads the camera definitions from the camera settings file
     */
    void load();

    /* Returns a list of the camera definitions loaded from the settings file,
     * or an empty list if the file has not yet been loaded or if an error
     * occurred when loading the file
     */
    const QVector<Camera>& getCameras();

private:
    QVector<Camera> _cameras;
};

} // namespace Soro

#endif // CAMERASETTINGSMODEL_H
