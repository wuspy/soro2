#ifndef VIDEOFORMAT_H
#define VIDEOFORMAT_H

#include <QtCore>
#include "mediaformat.h"

namespace Soro {

class VideoFormat: public MediaFormat {
    Q_GADGET
public:

    enum Encoding {
        Encoding_Null = -1,
        Encoding_MPEG2,
        Encoding_MJPEG,
        Encoding_H264,
        Encoding_VP8,
        Encoding_H265
    };
    Q_ENUM(Encoding)

    enum Resolution {
        Resolution_176_144 = 0,
        Resolution_432_240,
        Resolution_640x360,
        Resolution_1024x576,
        Resolution_1152x648,
        Resolution_1280x720,
        Resolution_1600x900,
        Resolution_1920x1080,
        Resolution_2560x1440,
        Resolution_3840x2160
    };
    Q_ENUM(Resolution)

    enum StereoMode {
        StereoMode_None = 0,
        StereoMode_SideBySide
    };
    Q_ENUM(StereoMode)

    VideoFormat();
    VideoFormat(const VideoFormat& other);
    VideoFormat(VideoFormat::Encoding encoding, VideoFormat::Resolution resolution, quint32 bitrate,
                quint32 framerate=0, StereoMode stereo=StereoMode_None, quint32 mjpegQuality=50, quint32 maxThreads=0);

    ~VideoFormat();

    QString toHumanReadableString() const Q_DECL_OVERRIDE;
    QString createGstEncodingArgs() const Q_DECL_OVERRIDE;
    QString createGstDecodingArgs(DecodingType type=DecodingType_Full) const Q_DECL_OVERRIDE;

    VideoFormat::Encoding getEncoding() const;
    VideoFormat::Resolution getResolution() const;
    quint32 getBitrate() const;
    quint32 getFramerate() const;
    VideoFormat::StereoMode getStereoMode() const;
    quint32 getMaxThreads() const;
    quint32 getMjpegQuality() const;

    void setEncoding(VideoFormat::Encoding encoding);
    void setResolution(VideoFormat::Resolution resolution);
    void setBitrate(quint32 bitrate);
    void setFramerate(quint32 framerate);
    void setStereoMode(VideoFormat::StereoMode stereo);
    void setMaxThreads(quint32 maxThreads);
    void setMjpegQuality(quint32 quality);

    quint32 getWidth() const;
    quint32 getHeight() const;

    bool isUseable() const Q_DECL_OVERRIDE;

    QString serialize() const Q_DECL_OVERRIDE;
    void deserialize(QString serial) Q_DECL_OVERRIDE;

    inline bool operator==(const VideoFormat& other) {
        return (_encoding == other._encoding) &&
                (_resolution == other._resolution) &&
                (_stereoMode == other._stereoMode) &&
                (_bitrate == other._bitrate) &&
                (_framerate == other._framerate) &&
                (_maxThreads == other._maxThreads) &&
                (_mjpegQuality == other._mjpegQuality);
    }

    inline bool operator!=(const VideoFormat& other) {
        return !(*this==other);
    }

protected:
    VideoFormat::Encoding _encoding;
    VideoFormat::Resolution _resolution;
    VideoFormat::StereoMode _stereoMode;
    quint32 _bitrate;
    quint32 _maxThreads;
    quint32 _framerate;
    qint32 _mjpegQuality;

    quint32 getResolutionHeight() const;
    quint32 getResolutionWidth() const;
};

} // namespace Soro

Q_DECLARE_METATYPE(Soro::VideoFormat)

#endif // VIDEOFORMAT_H
