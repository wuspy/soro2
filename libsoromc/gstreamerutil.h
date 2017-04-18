#ifndef GSTREAMERUTIL_H
#define GSTREAMERUTIL_H

#include <QString>
#include <QHostAddress>

/* This namespace has some useful functions for constructing GStreamer pipeline descriptions, as well as
 * other GStreamer-related functionality
 */
namespace Soro {
namespace GStreamerUtil {

/* Creates a pipeline string that accepts an RTP video stream on a UDP port, and decodes it from the specified codec to a raw video stream
 */
QString createRtpVideoDecodeBinString(QHostAddress address, quint16 port, uint codec, bool vaapi=false);

/* Creates a pipeline string that outputs a video test pattern
 */
QString createVideoTestSrcBinString(QString pattern="snow", uint width=640, uint height=480, uint framerate=60);

/* Creates a pipeline string that accepts an RTP audio stream on a UDP port, and decodes it from the specified codec to a raw audio stream
 */
QString createRtpAudioDecodeBinString(QHostAddress address, quint16 port, uint codec);

/* Creates a pipeline string that accepts an RTP audio stream on a UDP port, decodes it from the specified codec into a raw audio stream,
 * and plays it using autoaudiosink
 */
QString createRtpAudioPlayBinString(QHostAddress address, quint16 port, uint codec);

/* Creates a pipeline string that accepts an RTP stream on a UDP port, and depayloads it to an encoded video stream
 */
QString createRtpDepayBinString(QHostAddress address, quint16 port, uint codec);

/* Gets the element name and associated caps to RTP depayload a stream in the specified audio or video codec
 */
QString getCodecRtpDepayElement(uint codec);

/* Gets the element name and associated options to decode the specified video codec
 */
QString getVideoCodecDecodeElement(uint codec, bool vaapi=false);

/* Gets the element name and associated options to decode the specified audio codec
 */
QString getAudioCodecDecodeElement(uint codec);

/* Gets the human-readable name of a codec
 */
QString getCodecName(uint codec);

} // namespace GStreamerUtil
} // namespace Soro

#endif // GSTREAMERUTIL_H
