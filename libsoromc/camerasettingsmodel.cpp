/*
 * Copyright 2017 The University of Oklahoma.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
    QJsonArray jsonCamerasArray;
    QJsonParseError jsonError;
    QFile file(SORO_CAMERA_SETTINGS_FILE);

    _cameras.clear();

    if (!file.exists())
    {
        throw QString("The camera settings file \"%1\" does not exist.").arg(SORO_CAMERA_SETTINGS_FILE);
    }
    if (!file.open(QIODevice::ReadOnly))
    {
        throw QString("Error opening camera settings file \"%1\".").arg(SORO_CAMERA_SETTINGS_FILE);
    }

    rawJson = file.readAll();
    jsonDocument = QJsonDocument::fromJson(rawJson, &jsonError);
    if (jsonError.error != QJsonParseError::NoError)
    {
        throw QString("Error parsing camera settings file \"%1\": %2").arg(SORO_CAMERA_SETTINGS_FILE, jsonError.errorString());
    }

    if (!jsonDocument.object().contains("cameras"))
    {
        throw QString("Error parsing camera settings file: Item \"camera\" not found.");
    }

    jsonCamerasArray = jsonDocument.object()["cameras"].toArray();
    Q_FOREACH (QJsonValue jsonCamera, jsonCamerasArray)
    {
        Camera camera;
        camera.name = jsonCamera.toObject()["name"].toString();
        camera.id = jsonCamera.toObject()["id"].toInt(-1);
        camera.serial = jsonCamera.toObject()["matchSerial"].toString();
        camera.productId = jsonCamera.toObject()["matchProductId"].toString();
        camera.vendorId = jsonCamera.toObject()["matchVendorId"].toString();

        if (camera.id == -1)
        {
            throw QString("Error parsing camera settings file \"%1\": Camera entry is missing an id.").arg(SORO_CAMERA_SETTINGS_FILE);
        }
        _cameras.insert(camera.id, camera);
    }
}

const QList<CameraSettingsModel::Camera> CameraSettingsModel::getCameras() const
{
    return _cameras.values();
}

int CameraSettingsModel::getCameraIndexById(int id) const
{
    QList<CameraSettingsModel::Camera> cameras = getCameras();
    for (int i = 0; i < cameras.count(); ++i)
    {
        if (cameras[i].id == id) return i;
    }
    return -1;
}

int CameraSettingsModel::getCameraCount() const
{
    return _cameras.count();
}

} // namespace Soro
