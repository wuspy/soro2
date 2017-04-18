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

#define FILE_PATH SORO_SETTINGS_DIR + "/cameras.json"

namespace Soro {

void CameraSettingsModel::load() {
    QByteArray rawJson;
    QJsonDocument jsonDocument;
    QJsonArray jsonCamerasArray;
    QJsonArray jsonGroupsArray;
    QJsonParseError jsonError;
    QFile file(FILE_PATH);

    _cameras.clear();

    if (!file.exists())
    {
        throw QString("The camera settings file \"%1\" does not exist.").arg(FILE_PATH);
    }
    if (!file.open(QIODevice::ReadOnly))
    {
        throw QString("Error opening camera settings file \"%1\".").arg(FILE_PATH);
    }

    rawJson = file.readAll();
    jsonDocument = QJsonDocument::fromJson(rawJson, &jsonError);
    if (jsonError.error != QJsonParseError::NoError)
    {
        throw QString("Error parsing camera settings file \"%1\": %2").arg(FILE_PATH, jsonError.errorString());
    }

    if (!jsonDocument.object().contains("cameras"))
    {
        throw QString("Error parsing camera settings file: Item \"camera\" not found.");
    }

    QMap<int, Camera> cameraMap;

    jsonCamerasArray = jsonDocument.object()["cameras"].toArray();
    Q_FOREACH (QJsonValue jsonCamera, jsonCamerasArray)
    {
        Camera camera;
        int index = jsonCamera.toObject()["index"].toInt(-1);
        camera.name = jsonCamera.toObject()["name"].toString("");
        camera.serial = jsonCamera.toObject()["matchSerial"].toString();
        camera.productId = jsonCamera.toObject()["matchProductId"].toString();
        camera.vendorId = jsonCamera.toObject()["matchVendorId"].toString();

        if (index < 0)
        {
            throw QString("Error parsing camera settings file '%1': Camera entry has an invalid index.").arg(FILE_PATH);
        }
        if (camera.name.isEmpty())
        {
            throw QString("Error parsing camera settings file '%1': Camera entry is missing a name.").arg(FILE_PATH);
        }
        if (cameraMap.contains(index))
        {
            throw QString("Error parsing camera settings file '%1': Two cameras have a duplicate index entry. This is not allowed.").arg(FILE_PATH);
        }
        cameraMap.insert(index, camera);
    }

    // Transfer sorted map values to array
    _cameras = cameraMap.values();
}

CameraSettingsModel::Camera CameraSettingsModel::getCamera(uint index) const
{
    return _cameras.value(index);
}

int CameraSettingsModel::getCameraCount() const
{
    return _cameras.count();
}

} // namespace Soro
