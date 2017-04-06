#ifndef AUDIOFORMAT_H
#define AUDIOFORMAT_H

#include <QtCore>
#include "mediaformat.h"

namespace Soro {

class AudioFormat: public MediaFormat {
    Q_GADGET
public:

    enum Encoding {
        Encoding_Null = 0,
        Encoding_AC3
    };

    AudioFormat();
    AudioFormat(const AudioFormat& other);
    AudioFormat(AudioFormat::Encoding encoding, quint32 bitrate);

    ~AudioFormat();

    QString toHumanReadableString() const Q_DECL_OVERRIDE;
    QString createGstEncodingArgs() const Q_DECL_OVERRIDE;
    QString createGstDecodingArgs(DecodingType type=DecodingType_Full) const Q_DECL_OVERRIDE;

    AudioFormat::Encoding getEncoding() const;
    quint32 getBitrate() const;

    bool isUseable() const Q_DECL_OVERRIDE;

    void setEncoding(AudioFormat::Encoding encoding);
    void setBitrate(quint32 bitrate);

    QString serialize() const Q_DECL_OVERRIDE;
    void deserialize(QString serial) Q_DECL_OVERRIDE;

    inline bool operator==(const AudioFormat& other) {
        return (_encoding == other._encoding) &&
                (_bitrate == other._bitrate);
    }

    inline bool operator!=(const AudioFormat& other) {
        return !(*this==other);
    }

private:
    AudioFormat::Encoding _encoding;
    quint32 _bitrate;
};

} // namespace Soro

Q_DECLARE_METATYPE(Soro::AudioFormat)

#endif // AUDIOFORMAT_H
