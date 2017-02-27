#include "camerasettingsmodel.h"
#include "constants.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QFile>

namespace Soro {

void CameraSettingsModel::load() {
    QByteArray rawJson;
    QJsonDocument jsonDocument;
    QJsonObject jsonRootObject;
    QJsonObject jsonCamerasArray;
    QJsonParseError jsonError;
    QFile file(SORO_CAMERA_SETTINGS_FILE);

    _cameras.clear();

    if (!file.exists()) {
        throw SoroException(QString("The camera settings file \"%1\" does not exist.").arg(SORO_CAMERA_SETTINGS_FILE));
    }
    if (!file.open(QIODevice::ReadOnly)) {
        throw SoroException(QString("Error opening camera settings file \"%1\".").arg(SORO_CAMERA_SETTINGS_FILE));
    }

    rawJson = file.readAll();
    jsonDocument = QJsonDocument::fromJson(rawJson, &jsonError);
    if (jsonError.error != QJsonParseError::NoError) {
        throw SoroException(QString("Error parsing camera settings file \"%1\: %2").arg(SORO_CAMERA_SETTINGS_FILE, jsonError.errorString()));
    }

    if (!jsonDocument.object().contains("cameras")) {
        throw SoroException("Error parsing camera settings file: Item \"camera\" not found.");
    }

    jsonCamerasArray = jsonDocument.object()["cameras"].toObject();
    foreach (QJsonValue jsonCamera, jsonCamerasArray) {
        Camera camera;
        camera.name = jsonCamera.toObject()["name"].toString();
        camera.id = jsonCamera.toObject()["id"].toInt(-1);
        camera.serial = jsonCamera.toObject()["matchSerial"].toString();
        camera.productId = jsonCamera.toObject()["matchProductId"].toString();
        camera.vendorId = jsonCamera.toObject()["matchVendorId"].toString();

        if (camera.id == -1) {
            throw SoroException(QString("Error parsing camera settings file \"%1\": Camera entry is missing an id.").arg(SORO_CAMERA_SETTINGS_FILE));
        }
        _cameras.append(camera);
    }
}

const QVector<CameraSettingsModel::Camera>& CameraSettingsModel::getCameras() {
    return _cameras;
}

} // namespace Soro
