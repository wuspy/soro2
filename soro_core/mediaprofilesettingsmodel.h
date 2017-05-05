#ifndef MEDIAPROFILESETTINGSMODEL_H
#define MEDIAPROFILESETTINGSMODEL_H

#include <QString>
#include <QMap>
#include <QList>

#include "soro_core_global.h"
#include "gstreamerutil.h"

namespace Soro {

/* Settings loader for the audio/video profile definition file. This file is a JSON formatted array
 * specifying information about the different audio/video encoding profiles that will be available.
 */
class SORO_CORE_SHARED_EXPORT MediaProfileSettingsModel
{
    Q_GADGET
public:

    /* Loads the media profile definitions from the settings file. This will
     * throw an exception of type QString if an error occurrs
     */
    void load();

    /* Gets the video profile at the specified index
     */
    GStreamerUtil::VideoProfile getVideoProfile(uint index) const;

    QString getVideoProfileName(uint index) const;
    QString getVideoProfileName(GStreamerUtil::VideoProfile profile) const;

    /* Gets the audio profile at the specified index
     */
    GStreamerUtil::AudioProfile getAudioProfile(uint index) const;

    QString getAudioProfileName(uint index) const;
    QString getAudioProfileName(GStreamerUtil::AudioProfile profile) const;

    /* Gets the number of video profiles
     */
    int getVideoProfileCount() const;

    /* Gets the number of audio profiles
     */
    int getAudioProfileCount() const;

private:
    QList<GStreamerUtil::VideoProfile> _videoProfiles;
    QList<QString> _videoProfileNames;
    QList<GStreamerUtil::AudioProfile> _audioProfiles;
    QList<QString> _audioProfileNames;
};

} // namespace Soro

#endif // MEDIAPROFILESETTINGSMODEL_H
