#ifndef GSTREAMERUTIL_H
#define GSTREAMERUTIL_H

#include <QString>
#include <QHostAddress>

namespace Soro {
namespace GStreamerUtil {

/* Creates a pipeline string that accepts an RTP video stream on a UDP port, and decodes it from the specified codec to a raw video stream
 */
QString createRtpVideoDecodeBinString(QHostAddress address, quint16 port, int codec, bool vaapi=false);

/* Creates a pipeline string that accepts an RTP audio stream on a UDP port, and decodes it from the specified codec to a raw audio stream
 */
QString createRtpAudioDecodeBinString(QHostAddress address, quint16 port, int codec);

/* Creates a pipeline string that accepts an RTP stream on a UDP port, and depayloads it to an encoded video stream
 */
QString createRtpDepayBinString(QHostAddress address, quint16 port, int codec);

/* Gets the element name and associated caps to RTP depayload a stream in the specified audio or video codec
 */
QString getCodecRtpDepayElement(int codec);

/* Gets the element name and associated options to decode the specified video codec
 */
QString getVideoCodecDecodeElement(int codec, bool vaapi=false);

/* Gets the element name and associated options to decode the specified audio codec
 */
QString getAudioCodecDecodeElement(int codec);

} // namespace GStreamerUtil
} // namespace Soro

#endif // GSTREAMERUTIL_H
