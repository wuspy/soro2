#ifndef MEDIAPROFILESETTINGSMODEL_H
#define MEDIAPROFILESETTINGSMODEL_H

#include <QString>
#include <QMap>
#include <QList>

#include "soro_core_global.h"

namespace Soro {

/* Settings loader for the audio/video profile definition file. This file is a JSON formatted array
 * specifying information about the different audio/video encoding profiles that will be available.
 */
class SORO_CORE_SHARED_EXPORT MediaProfileSettingsModel
{
    Q_GADGET
public:
    /* This struct represents a single video profile definition
     */
    struct VideoProfile {
        QString name;
        quint8 codec;
        uint width;
        uint height;
        uint bitrate;
        uint framerate;
        uint quality;
    };

    /* This struct represents a single audio profile definition
     */
    struct AudioProfile
    {
        QString name;
        quint8 codec;
        uint bitrate;
    };

    /* Loads the media profile definitions from the settings file. This will
     * throw an exception of type QString if an error occurrs
     */
    void load();

    /* Gets the video profile at the specified index
     */
    VideoProfile getVideoProfile(uint index) const;

    /* Gets the audio profile at the specified index
     */
    AudioProfile getAudioProfile(uint index) const;

    /* Gets the number of video profiles
     */
    int getVideoProfileCount() const;

    /* Gets the number of audio profiles
     */
    int getAudioProfileCount() const;

private:
    QList<MediaProfileSettingsModel::VideoProfile> _videoProfiles;
    QList<MediaProfileSettingsModel::AudioProfile> _audioProfiles;
};

} // namespace Soro

#endif // MEDIAPROFILESETTINGSMODEL_H
