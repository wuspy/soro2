/*
 * Copyright 2017 Jacob Jordan <doublejinitials@ou.edu>
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

#include "mediaprofilesettingsmodel.h"
#include "constants.h"
#include "gstreamerutil.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QFile>

#define FILE_PATH SORO_MC_SETTINGS_DIR + "/media_profiles.json"

namespace Soro {

void MediaProfileSettingsModel::load() {
    QByteArray rawJson;
    QJsonDocument jsonDocument;
    QJsonArray jsonVideoProfilesArray;
    QJsonParseError jsonError;
    QFile file(FILE_PATH);

    _videoProfiles.clear();
    _audioProfiles.clear();

    if (!file.exists())
    {
        throw QString("The media profile settings file \"%1\" does not exist.").arg(FILE_PATH);
    }
    if (!file.open(QIODevice::ReadOnly))
    {
        throw QString("Error opening media profile settings file \"%1\".").arg(FILE_PATH);
    }

    rawJson = file.readAll();
    jsonDocument = QJsonDocument::fromJson(rawJson, &jsonError);
    if (jsonError.error != QJsonParseError::NoError)
    {
        throw QString("Error parsing media profile settings file \"%1\": %2").arg(FILE_PATH, jsonError.errorString());
    }

    if (!jsonDocument.object().contains("video_profiles"))
    {
        throw QString("Error parsing media profile settings file \"%1\": Item \"video_profiles\" not found.").arg(FILE_PATH);
    }

    if (!jsonDocument.object().contains("audio_profiles"))
    {
        throw QString("Error parsing media profile settings file \"%1\": Item \"audio_profiles\" not found.").arg(FILE_PATH);
    }

    //
    // Parse video profiles
    //

    // Profiles will be inserted into this map by the 'index' value so that they will be sorted
    QMap<int, GStreamerUtil::VideoProfile> videoProfileMap;
    QMap<int, QString> videoProfileNameMap;

    jsonVideoProfilesArray = jsonDocument.object()["video_profiles"].toArray();
    Q_FOREACH (QJsonValue jsonObject, jsonVideoProfilesArray)
    {
        GStreamerUtil::VideoProfile profile;
        int index = jsonObject.toObject()["index"].toInt(-1);
        QString profileName = jsonObject.toObject()["name"].toString("");
        profile.width = jsonObject.toObject()["width"].toInt(0);
        profile.height = jsonObject.toObject()["height"].toInt(0);
        profile.bitrate = jsonObject.toObject()["bitrate"].toInt(0);
        profile.framerate = jsonObject.toObject()["framerate"].toInt(0);
        profile.mjpeg_quality = jsonObject.toObject()["quality"].toInt(0);
        QString encoding = jsonObject.toObject()["encoding"].toString().toLower();

        if (encoding == "mjpeg")
        {
            profile.codec = GStreamerUtil::VIDEO_CODEC_MJPEG;
        }
        else if (encoding == "mp4")
        {
            profile.codec = GStreamerUtil::VIDEO_CODEC_MPEG4;
        }
        else if (encoding == "mp2")
        {
            profile.codec = GStreamerUtil::VIDEO_CODEC_MPEG2;
        }
        else if (encoding == "vp8")
        {
            profile.codec = GStreamerUtil::VIDEO_CODEC_VP8;
        }
        else if (encoding == "vp9")
        {
            profile.codec = GStreamerUtil::VIDEO_CODEC_VP9;
        }
        else if (encoding == "h264")
        {
            profile.codec = GStreamerUtil::VIDEO_CODEC_H264;
        }
        else if (encoding == "h265")
        {
            profile.codec = GStreamerUtil::VIDEO_CODEC_H265;
        }
        else
        {
            throw QString("Erro Ar parsing media profile settings file \"%1\": Unknown value for \"encoding\" on video profile.").arg(FILE_PATH);
        }

        if (index < 0)
        {
            throw QString("Error parsing media profile settings file '%1': Video profile entry has an invalid index.").arg(FILE_PATH);
        }
        videoProfileMap.insert(index, profile);
        videoProfileNameMap.insert(index, profileName);
    }

    // Transfer sorted map values to array
    _videoProfiles = videoProfileMap.values();
    _videoProfileNames = videoProfileNameMap.values();

    //
    // Parse audio profiles
    //

    // Profiles will be inserted into this map by the 'index' value so that they will be sorted
    QMap<int, GStreamerUtil::AudioProfile> audioProfileMap;
    QMap<int, QString> audioProfileNameMap;

    jsonVideoProfilesArray = jsonDocument.object()["audio_profiles"].toArray();
    Q_FOREACH (QJsonValue jsonObject, jsonVideoProfilesArray)
    {
        GStreamerUtil::AudioProfile profile;
        int index = jsonObject.toObject()["index"].toInt(-1);
        QString profileName = jsonObject.toObject()["name"].toString("");
        profile.bitrate = jsonObject.toObject()["bitrate"].toInt(0);
        QString encoding = jsonObject.toObject()["encoding"].toString().toLower();

        if (encoding == "ac3")
        {
            profile.codec = GStreamerUtil::AUDIO_CODEC_AC3;
        }
        else
        {
            throw QString("Error parsing media profile settings file \"%1\": Unknown value for \"encoding\" on audio profile.").arg(FILE_PATH);
        }

        if (index < 0)
        {
            throw QString("Error parsing media profile settings file '%1': Audio profile entry has an invalid index.").arg(FILE_PATH);
        }
        audioProfileMap.insert(index, profile);
        audioProfileNameMap.insert(index, profileName);
    }

    // Transfer sorted map values to array
    _audioProfiles = audioProfileMap.values();
    _audioProfileNames = audioProfileNameMap.values();
}

GStreamerUtil::VideoProfile MediaProfileSettingsModel::getVideoProfile(uint index) const
{
    return _videoProfiles.value(index);
}

int MediaProfileSettingsModel::getVideoProfileCount() const
{
    return _videoProfiles.count();
}

GStreamerUtil::AudioProfile MediaProfileSettingsModel::getAudioProfile(uint index) const
{
    return _audioProfiles.value(index);
}

int MediaProfileSettingsModel::getAudioProfileCount() const
{
    return _audioProfiles.count();
}

QString MediaProfileSettingsModel::getVideoProfileName(uint index) const
{
    return _videoProfileNames.value(index);
}

QString MediaProfileSettingsModel::getVideoProfileName(GStreamerUtil::VideoProfile profile) const
{
    for (int i = 0; i < _videoProfiles.size(); ++i)
    {
        if (_videoProfiles.value(i) == profile) return _videoProfileNames.value(i);
    }
    return "";
}

QString MediaProfileSettingsModel::getAudioProfileName(uint index) const
{
    return _audioProfileNames.value(index);
}

QString MediaProfileSettingsModel::getAudioProfileName(GStreamerUtil::AudioProfile profile) const
{
    for (int i = 0; i < _audioProfiles.size(); ++i)
    {
        if (_audioProfiles.value(i) == profile) return _audioProfileNames.value(i);
    }
    return "";
}

} // namespace Soro
