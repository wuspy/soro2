#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QStringList>

#include "soro_core_global.h"

#define LOG_D(tag, msg) Soro::Logger::logDebug(tag, msg)
#define LOG_I(tag, msg) Soro::Logger::logInfo(tag, msg)
#define LOG_W(tag, msg) Soro::Logger::logWarn(tag, msg)
#define LOG_E(tag, msg) Soro::Logger::logError(tag, msg)

namespace Soro {

/* Singleton logging class for the application. All logging (Logger::log...) functions are thread-safe.
 */
class SORO_CORE_EXPORT Logger
{
public:
    enum LogLevel {
        LogLevelDisabled = 0,
        LogLevelError,
        LogLevelWarning,
        LogLevelInfo,
        LogLevelDebug
    };

    static void logDebug(QString tag, QString message);
    static void logInfo(QString tag, QString message);
    static void logWarn(QString tag, QString message);
    static void logError(QString tag, QString message);
    static void setMaxLogLevel(LogLevel level);

private:

    // These format the log messages to the desiered text appearance
    QStringList _textFormat;
    QStringList _stdoutFormat;
    LogLevel _maxLevel;

    Logger();
    Logger(Logger const&)=delete;
    void operator=(Logger const&)=delete;
    void log(LogLevel level, QString tag, QString message);

    static Logger* getInstance();

    static Logger *_self;
};

}

#endif // LOGGER_H
