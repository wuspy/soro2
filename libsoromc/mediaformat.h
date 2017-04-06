#ifndef MEDIAFORMAT_H
#define MEDIAFORMAT_H

#include <QtCore>

namespace Soro {

class MediaFormat {
    Q_GADGET

public:

    enum DecodingType {
        /* Fully decodes the video to a raw video stream */
        DecodingType_Full = 0,
        /* Only decodes the RTP stream to an encoded video stream */
        DecodingType_RtpDecodeOnly
    };
    Q_ENUM(DecodingType)

    virtual ~MediaFormat() { }

    virtual QString toHumanReadableString() const=0;
    virtual QString createGstEncodingArgs() const=0;
    virtual QString createGstDecodingArgs(DecodingType type=DecodingType_Full) const=0;

    virtual bool isUseable() const=0;

    virtual QString serialize() const=0;
    virtual void deserialize(QString serial)=0;
};

} // namespace Soro

#endif // MEDIAFORMAT_H
