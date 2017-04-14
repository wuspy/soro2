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

#include "gstreamerutil.h"
#include "constants.h"

namespace Soro {
namespace GStreamerUtil {

QString createRtpVideoDecodeBinString(QHostAddress address, quint16 port, int codec, bool vaapi)
{
    return createRtpDepayBinString(address, port, codec) + " ! " + getVideoCodecDecodeElement(codec, vaapi);
}

QString createRtpDepayBinString(QHostAddress address, quint16 port, int codec)
{
    return QString("udpsrc address=%1 port=%2 ! %3").arg(
                address.toString(),
                QString::number(port),
                getCodecRtpDepayElement(codec));
}

QString getCodecRtpDepayElement(int codec)
{
    switch (codec)
    {
    case VIDEO_CODEC_MPEG4:
        return "application/x-rtp,media=video,encoding-name=MP4V-ES,clock-rate=90000,profile-level-id=1,payload=96 ! rtpmp4vdepay";
    case VIDEO_CODEC_H264:
        return "application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,payload=96 ! rtph264depay";
    case VIDEO_CODEC_MJPEG:
        return "application/x-rtp,media=video,encoding-name=JPEG,payload=26 ! rtpjpegdepay";
    case VIDEO_CODEC_VP8:
        return "application/x-rtp,media=video,encoding-name=VP8,clock-rate=90000,payload=96 ! rtpvp8depay";
    case VIDEO_CODEC_VP9:
        return "application/x-rtp,media=video,encoding-name=VP9,clock-rate=90000,payload=96 ! rtpvp9depay";
    case VIDEO_CODEC_H265:
        return "application/x-rtp,media=video,encoding-name=H265,clock-rate=90000,payload=96 ! rtph265depay";
    default:
        // unknown codec
        return "";
    }
}

QString getAudioCodecDecodeElement(int codec)
{
    switch (codec)
    {
    case AUDIO_CODEC_AC3:
        return "a52dec";
    case AUDIO_CODEC_MP3:
        return "avdec_mp3";
    case AUDIO_CODEC_VORBIS:
        return "vorbisdec";
    default:
        // unknown codec
        return "";
    }
}

QString getVideoCodecDecodeElement(int codec, bool vaapi)
{
    // Single decode element for all VAAPI codecs
    if (vaapi) return "vaapidecode";

    switch (codec)
    {
    case VIDEO_CODEC_MPEG4:
        return "avdec_mpeg4";
    case VIDEO_CODEC_H264:
        return "avdec_h264";
    case VIDEO_CODEC_MJPEG:
        return "jpegdec";
    case VIDEO_CODEC_VP8:
        return "avdec_vp8";
    case VIDEO_CODEC_VP9:
        return "avdec_vp9";
    case VIDEO_CODEC_H265:
        return "avdec_h265";
    default:
        // unknown codec
        return "";
    }
}

} // namespace GStreamerUtil
} // namespace Soro
