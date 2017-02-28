#include "camerasettingsmodel.h"
#include "constants.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QFile>

namespace Soro {

bool CameraSettingsModel::load() {
    QByteArray rawJson;
    QJsonDocument jsonDocument;
    QJsonArray jsonCamerasArray;
    QJsonParseError jsonError;
    QFile file(SORO_CAMERA_SETTINGS_FILE);

    _cameras.clear();

    if (!file.exists())
    {
        _errorString = QString("The camera settings file \"%1\" does not exist.").arg(SORO_CAMERA_SETTINGS_FILE);
        return false;
    }
    if (!file.open(QIODevice::ReadOnly))
    {
        _errorString = QString("Error opening camera settings file \"%1\".").arg(SORO_CAMERA_SETTINGS_FILE);
        return false;
    }

    rawJson = file.readAll();
    jsonDocument = QJsonDocument::fromJson(rawJson, &jsonError);
    if (jsonError.error != QJsonParseError::NoError)
    {
        _errorString = QString("Error parsing camera settings file \"%1\": %2").arg(SORO_CAMERA_SETTINGS_FILE, jsonError.errorString());
        return false;
    }

    if (!jsonDocument.object().contains("cameras"))
    {
        _errorString = "Error parsing camera settings file: Item \"camera\" not found.";
        return false;
    }

    jsonCamerasArray = jsonDocument.object()["cameras"].toArray();
    foreach (QJsonValue jsonCamera, jsonCamerasArray)
    {
        Camera camera;
        camera.name = jsonCamera.toObject()["name"].toString();
        camera.id = jsonCamera.toObject()["id"].toInt(-1);
        camera.serial = jsonCamera.toObject()["matchSerial"].toString();
        camera.productId = jsonCamera.toObject()["matchProductId"].toString();
        camera.vendorId = jsonCamera.toObject()["matchVendorId"].toString();

        if (camera.id == -1)
        {
            _errorString = QString("Error parsing camera settings file \"%1\": Camera entry is missing an id.").arg(SORO_CAMERA_SETTINGS_FILE);
            return false;
        }
        _cameras.append(camera);
    }
    return true;
}

QString CameraSettingsModel::errorString() const
{
    return _errorString;
}

const QVector<CameraSettingsModel::Camera>& CameraSettingsModel::getCameras() const
{
    return _cameras;
}

} // namespace Soro
