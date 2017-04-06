#include "audioformat.h"
#include "logger.h"

#include <QJsonObject>
#include <QJsonDocument>

#define LOG_TAG "AudioFormat"

namespace Soro {

AudioFormat::AudioFormat() {
    // Default
    _encoding = Encoding_Null;
    _bitrate = 32000;
}

AudioFormat::AudioFormat(const AudioFormat &other) {
    _encoding = other._encoding;
    _bitrate = other._bitrate;
}

AudioFormat::AudioFormat(AudioFormat::Encoding encoding, quint32 bitrate) {
    _encoding = encoding;
    _bitrate = bitrate;
}

AudioFormat::~AudioFormat() { }

bool AudioFormat::isUseable() const {
    return (_encoding != Encoding_Null) &&
            (_bitrate > 0);
}

quint32 AudioFormat::getBitrate() const {
    return _bitrate;
}

void AudioFormat::setEncoding(AudioFormat::Encoding encoding) {
    _encoding = encoding;
}

void AudioFormat::setBitrate(quint32 bitrate) {
    _bitrate = bitrate;
}

QString AudioFormat::toHumanReadableString() const {
    QString str = "%1@%2";
    QString encoding;
    switch (_encoding) {
    case Encoding_Null:
        return "No Audio";
    case Encoding_AC3:
        encoding = "AC3";
        break;
    default:
        return "Unknown Encoding";
    }

    return str.arg(encoding,
                   QString::number(_bitrate));
}

QString AudioFormat::createGstEncodingArgs() const {
    QString encString =  "audioconvert ! "
                         "audio/x-raw,rate=%1 ! ";

    switch (_encoding) {
    case Encoding_AC3:
        encString += "avenc_ac3 ! "
                     "rtpac3pay";
        break;
    default:
        //unknown codec
        Logger::logError(LOG_TAG, "Unknown audio encoding");
        return "";
    }

    encString = encString.arg(QString::number(_bitrate));
    return encString;
}

QString AudioFormat::createGstDecodingArgs(DecodingType type) const {
    switch (_encoding) {
    case Encoding_AC3:
        return QString("application/x-rtp,media=audio,clock-rate=44100,encoding-name=AC3 ! "
                        " ! rtpac3depay")
                + ((type != DecodingType_RtpDecodeOnly) ?
                        " ! a52dec" : "");
    default:
        // unknown codec
        Logger::logError(LOG_TAG, "Unknown audio encoding");
        return "";
    }
}

QString AudioFormat::serialize() const {
    QString serial;
    serial += QString::number(static_cast<qint32>(_encoding)) + "_";
    serial += QString::number(_bitrate);

    return serial;
}

void AudioFormat::deserialize(QString serial) {
    QStringList items = serial.split('_');
    if (items.size() < 2) {
        Logger::logError(LOG_TAG, "deserialize(): Invalid string");
        return;
    }

    bool ok;
    reinterpret_cast<qint32&>(_encoding) = items[0].toUInt(&ok);
    if (!ok) {
        Logger::logError(LOG_TAG, "deserialize(): Invalid option for encoding");
    }
    _bitrate = items[1].toUInt(&ok);
    if (!ok) {
        Logger::logError(LOG_TAG, "deserialize(): Invalid option for bitrate");
    }
}

} // namespace Soro
