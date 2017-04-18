#include "videoformat.h"
#include "logger.h"

#define LOG_TAG "VideoFormat"

namespace Soro {

VideoFormat::VideoFormat() {
    // Default
    _encoding = Encoding_Null;
    _resolution = Resolution_640x360;
    _stereoMode = StereoMode_None;
    _bitrate = 1000000;
    _maxThreads = 3;
    _mjpegQuality = 50;
    _framerate = 0;
}

VideoFormat::VideoFormat(const VideoFormat &other) {
    _encoding = other._encoding;
    _resolution = other._resolution;
    _stereoMode = other._stereoMode;
    _bitrate = other._bitrate;
    _maxThreads = other._maxThreads;
    _mjpegQuality = other._mjpegQuality;
    _framerate = other._framerate;
}

VideoFormat::VideoFormat(VideoFormat::Encoding encoding, VideoFormat::Resolution resolution, quint32 bitrate,
                         quint32 framerate, StereoMode stereo, quint32 mjpegQuality, quint32 maxThreads) {
    _encoding = encoding;
    _resolution = resolution;
    _bitrate = bitrate;
    _framerate = framerate;
    _stereoMode = stereo;
    _maxThreads = maxThreads;
    _mjpegQuality = mjpegQuality;
}

VideoFormat::~VideoFormat() { }

bool VideoFormat::isUseable() const {
    return (_encoding != Encoding_Null) && (_bitrate > 0);
}

VideoFormat::Encoding VideoFormat::getEncoding() const {
    return _encoding;
}

VideoFormat::Resolution VideoFormat::getResolution() const {
    return _resolution;
}

quint32 VideoFormat::getBitrate() const {
    return _bitrate;
}

quint32 VideoFormat::getFramerate() const {
    return _framerate;
}

VideoFormat::StereoMode VideoFormat::getStereoMode() const {
    return _stereoMode;
}

quint32 VideoFormat::getMaxThreads() const {
    return _maxThreads;
}

quint32 VideoFormat::getMjpegQuality() const {
    return _mjpegQuality;
}

void VideoFormat::setEncoding(VideoFormat::Encoding encoding) {
    _encoding = encoding;
}

void VideoFormat::setResolution(VideoFormat::Resolution resolution) {
    _resolution = resolution;
}

void VideoFormat::setBitrate(quint32 bitrate) {
    _bitrate = bitrate;
}

void VideoFormat::setFramerate(quint32 framerate) {
    _framerate = framerate;
}

void VideoFormat::setStereoMode(VideoFormat::StereoMode stereo) {
    _stereoMode = stereo;
}

void VideoFormat::setMaxThreads(quint32 maxThreads) {
    _maxThreads = maxThreads;
}

void VideoFormat::setMjpegQuality(quint32 quality) {
    _mjpegQuality = quality;
}

quint32 VideoFormat::getResolutionWidth() const {
    switch (_resolution) {
    case Resolution_176_144:    return 176;
    case Resolution_432_240:    return 432;
    case Resolution_640x360:    return 640;
    case Resolution_1024x576:   return 1024;
    case Resolution_1152x648:   return 1152;
    case Resolution_1280x720:   return 1280;
    case Resolution_1600x900:   return 1600;
    case Resolution_1920x1080:  return 1920;
    case Resolution_2560x1440:  return 2560;
    case Resolution_3840x2160:  return 3840;
    default:
        Logger::logError(LOG_TAG, "Unknown _resolution value, returning 1280x720 default");
        return 1280;
    }
}

quint32 VideoFormat::getResolutionHeight() const {
    switch (_resolution) {
    case Resolution_176_144:    return 144;
    case Resolution_432_240:    return 240;
    case Resolution_640x360:    return 360;
    case Resolution_1024x576:   return 576;
    case Resolution_1152x648:   return 648;
    case Resolution_1280x720:   return 720;
    case Resolution_1600x900:   return 900;
    case Resolution_1920x1080:  return 1080;
    case Resolution_2560x1440:  return 1440;
    case Resolution_3840x2160:  return 2160;
    default:
        Logger::logError(LOG_TAG, "Unknown _resolution value, returning 1280x720 default");
        return 720;
    }
}

quint32 VideoFormat::getWidth() const {
    return _stereoMode == StereoMode_SideBySide ?
                getResolutionWidth() / 2 :
                getResolutionWidth();
}

quint32 VideoFormat::getHeight() const {
    return getResolutionHeight();
}

QString VideoFormat::toHumanReadableString() const {
    QString framerate;

    switch (_framerate) {
    case 0:
        framerate = "Auto";
        break;
    default:
        framerate = QString::number(_framerate) + "/1";
        break;
    }

    switch (_encoding) {
    case Encoding_Null:
        return "No Video";
    case Encoding_MPEG2:
        return QString("MPEG2 %1x%2@%3 (%4K)").arg(
                    QString::number(getResolutionWidth()),
                    QString::number(getResolutionHeight()),
                    framerate,
                    QString::number(_bitrate / 1000));
    case Encoding_MJPEG:
        return QString("MJPEG %1x%2@%3 (%4\%)").arg(
                    QString::number(getResolutionWidth()),
                    QString::number(getResolutionHeight()),
                    framerate,
                    QString::number(_mjpegQuality));
    case Encoding_H264:
        return QString("H264 %1x%2@%3 (%4K)").arg(
                    QString::number(getResolutionWidth()),
                    QString::number(getResolutionHeight()),
                    framerate,
                    QString::number(_bitrate / 1000));
    case Encoding_VP8:
        return QString("VP8 %1x%2@%3 (%4K)").arg(
                    QString::number(getResolutionWidth()),
                    QString::number(getResolutionHeight()),
                    framerate,
                    QString::number(_bitrate / 1000));
    case Encoding_H265:
        return QString("H265 %1x%2@%3 (%4K)").arg(
                    QString::number(getResolutionWidth()),
                    QString::number(getResolutionHeight()),
                    framerate,
                    QString::number(_bitrate / 1000));
    default:
        return "Unknown Encoding";
    }
}

QString VideoFormat::createGstEncodingArgs() const {
    QString encString = "";
    QString stereoEncString = "";
    QString framerateEncString = "";
    int bitrate = _bitrate;

    switch (_stereoMode) {
    case StereoMode_SideBySide:
        stereoEncString = "videoscale method=0 add-borders=false ! "
                          "video/x-raw,width=%1,height=%2 ! ";
        stereoEncString = stereoEncString.arg(QString::number(getWidth()),
                                              QString::number(getHeight()));
        bitrate /= 2;
        break;
    default:
        break;
    }

    if (_framerate > 0) {
        framerateEncString = QString(
                        "videorate ! "
                        "video/x-raw,framerate=%1/1 ! "
                    ).arg(
                        QString::number(_framerate)
                    );
    }

    // Comming encoding params for all codecs
    encString = QString(
                    "videoscale method=0 ! "
                    "video/x-raw,width=%1,height=%2 ! "
                    "%3" // For stereo
                    "%4" // For framerate
                    "videoconvert ! "
                ).arg(
                    QString::number(getResolutionWidth()),
                    QString::number(getResolutionHeight()),
                    stereoEncString,
                    framerateEncString
                );

    switch (_encoding) {
    case Encoding_MPEG2:
        encString += QString(
                        "avenc_mpeg4 bitrate=%1 bitrate-tolerance=%2 max-threads=%3 ! "
                        "rtpmp4vpay config-interval=3 pt=96"
                    ).arg(
                        QString::number(bitrate),  // mpeg2 has bitrate in bit/sec
                        QString::number(bitrate / 4),
                        QString::number(_maxThreads)
                    );
        break;
    case Encoding_MJPEG:
        encString += QString(
                        "jpegenc quality=%1 ! rtpjpegpay"
                    ).arg(
                        QString::number(_mjpegQuality)
                    );
        break;
    case Encoding_H264:
        encString += QString(
                        "x264enc tune=zerolatency bitrate=%1 threads=%2 ! "
                        "rtph264pay config-interval=3 pt=96"
                    ).arg(
                        QString::number(bitrate / 1000), // x264 has bitrate in kbit/sec
                        QString::number(_maxThreads)
                    );
        break;
    case Encoding_VP8:
        encString += QString(
                        "vp8enc target-bitrate=%1 threads=%2 ! "
                        "rtpvp8pay pt=96"
                    ).arg(
                        QString::number(bitrate), // vp8 has bitrate in bit/sec
                        QString::number(_maxThreads)
                    );
        break;
    case Encoding_H265:
        encString += QString(
                        "x265enc speed-preset=ultrafast tune=zerolatency bitrate=%1 ! "
                        "rtph265pay config-interval=3 pt=96"
                    ).arg(
                        QString::number(bitrate / 1000) // x265 has bitrate in kbit/sec
                    );
        break;
    default:
        //unknown codec
        Logger::logError(LOG_TAG, "Unknown video encoding");
        return "";
    }

    return encString;
}

QString VideoFormat::createGstDecodingArgs(VideoFormat::DecodingType type) const {
    switch (_encoding) {
    case Encoding_MPEG2:
        // Same decoding args for all MPEG2 formats
        return QString("application/x-rtp,media=video,encoding-name=MP4V-ES,clock-rate=90000,profile-level-id=1,payload=96"
                        " ! rtpmp4vdepay")
                + ((type != DecodingType_RtpDecodeOnly) ?
                        " ! avdec_mpeg4" : "");
    case Encoding_H264:
        return QString("application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,payload=96"
                        " ! rtph264depay")
                + ((type != DecodingType_RtpDecodeOnly) ?
                        " ! avdec_h264" : "");
    case Encoding_MJPEG:
        return QString("application/x-rtp,media=video,encoding-name=JPEG,payload=26"
                       " ! rtpjpegdepay")
                + ((type != DecodingType_RtpDecodeOnly) ?
                       " ! jpegdec" : "");
    case Encoding_VP8:
        return QString("application/x-rtp,media=video,encoding-name=VP8,clock-rate=90000,payload=96"
                       " ! rtpvp8depay")
                + ((type != DecodingType_RtpDecodeOnly) ?
                       " ! avdec_vp8" : "");
    case Encoding_H265:
        return QString("application/x-rtp,media=video,encoding-name=H265,clock-rate=90000,payload=96"
                       " ! rtph265depay")
                + ((type != DecodingType_RtpDecodeOnly) ?
                       " ! avdec_h265" : "");
    default:
        // unknown codec
        Logger::logError(LOG_TAG, "Unknown video encoding");
        return "";
    }
}

QString VideoFormat::serialize() const {
    QString serial;
    serial += QString::number(static_cast<qint32>(_encoding)) + "_";
    serial += QString::number(static_cast<qint32>(_resolution)) + "_";
    serial += QString::number(static_cast<qint32>(_stereoMode)) + "_";
    serial += QString::number(_framerate) + "_";
    serial += QString::number(_bitrate) + "_";
    serial += QString::number(_maxThreads) + "_";
    serial += QString::number(_mjpegQuality);
    return serial;
}

void VideoFormat::deserialize(QString serial) {
    QStringList items = serial.split('_');
    if (items.size() < 7) {
        Logger::logError(LOG_TAG, "deserialize(): Invalid string");
        return;
    }

    bool ok;
    reinterpret_cast<qint32&>(_encoding) = items[0].toInt(&ok);
    if (!ok) {
        Logger::logError(LOG_TAG, "deserialize(): Invalid option for encoding");
    }
    reinterpret_cast<qint32&>(_resolution) = items[1].toInt(&ok);
    if (!ok) {
        Logger::logError(LOG_TAG, "deserialize(): Invalid option for resolution");
    }
    reinterpret_cast<qint32&>(_stereoMode) = items[2].toInt(&ok);
    if (!ok) {
        Logger::logError(LOG_TAG, "deserialize(): Invalid option for stereo mode");
    }
    _framerate = items[3].toUInt(&ok);
    if (!ok) {
        Logger::logError(LOG_TAG, "deserialize(): Invalid option for framerate");
    }
    _bitrate = items[4].toUInt(&ok);
    if (!ok) {
        Logger::logError(LOG_TAG, "deserialize(): Invalid option for bitrate");
    }
    _maxThreads = items[5].toUInt(&ok);
    if (!ok) {
        Logger::logError(LOG_TAG, "deserialize(): Invalid option for max threads");
    }
    _mjpegQuality = items[6].toUInt(&ok);
    if (!ok) {
        Logger::logError(LOG_TAG, "deserialize(): Invalid option for mjpeg quality");
    }
}

} // namespace Soro
