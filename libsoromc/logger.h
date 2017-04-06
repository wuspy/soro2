#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>

namespace Soro {

class Logger
{
    Q_GADGET

public:
    static void logDebug(QString tag, QString message);
    static void logInfo(QString tag, QString message);
    static void logWarn(QString tag, QString message);
    static void logError(QString tag, QString message);

private:
    enum LogLevel {
        LogLevelDebug,
        LogLevelInfo,
        LogLevelWarning,
        LogLevelError
    };

    Logger();
    Logger(Logger const&)=delete;
    void operator=(Logger const&)=delete;
    void log(LogLevel level, QString tag, QString message);

    static Logger* getInstance();

    static Logger *_self;
};

}

#endif // LOGGER_H
