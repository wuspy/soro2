#ifndef SOROEXCEPTION_H
#define SOROEXCEPTION_H

#include <QException>

namespace Soro {

/* A general exception that can be used for any error that may occur
 * in this library. Allows for a custom error message.
 */
class SoroException: public QException {
public:
    SoroException(QString message) { _message = message.toLocal8Bit().constData(); }
    SoroException(const char *message) { _message = message; }
    void raise() const { throw *this; }
    SoroException *clone() const { return new SoroException(*this); }
    const char *what() const noexcept { return _message ? _message : ""; }

private:
    const char *_message;
};

} // namespace Soro

#endif // SOROEXCEPTION_H
