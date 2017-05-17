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

#include "logger.h"

#include <QTextStream>
#include <QTime>

namespace Soro {

Logger *Logger::_self = nullptr;

Logger::Logger()
{
    // create default text formatting
    _stdoutFormat << "\033[31m[E]\033[0m %1 \033[35m%2\033[0m: %3";
    _stdoutFormat << "\033[33m[W]\033[0m %1 \033[35m%2\033[0m: %3";
    _stdoutFormat << "\033[34m[I]\033[0m %1 \033[35m%2\033[0m: %3";
    _stdoutFormat << "[D] %1 \033[35m%2\033[0m: %3";

    // default unless later set otherwise
    _textFormat << "[E]\t%1\t%2:\t%3";
    _textFormat << "[W]\t%1\t%2:\t%3";
    _textFormat << "[I]\t%1\t%2:\t%3";
    _textFormat << "[D]\t%1\t%2:\t%3";

    _maxLevel = LogLevelDebug;
}

Logger* Logger::getInstance()
{
    if (!_self)
    {
        _self = new Logger();
    }
    return _self;
}

void Logger::logDebug(QString tag, QString message)
{
    getInstance()->log(LogLevelDebug, tag, message);
}

void Logger::logInfo(QString tag, QString message)
{
    getInstance()->log(LogLevelInfo, tag, message);
}

void Logger::logWarn(QString tag, QString message)
{
    getInstance()->log(LogLevelWarning, tag, message);
}

void Logger::logError(QString tag, QString message)
{
    getInstance()->log(LogLevelError, tag, message);
}

void Logger::setMaxLogLevel(LogLevel level)
{
    getInstance()->_maxLevel = level;
}

void Logger::log(LogLevel level, QString tag, QString message) {
    if (level <= _maxLevel) {
        QString formatted = _stdoutFormat[reinterpret_cast<int&>(level) - 1].arg(QTime::currentTime().toString(), tag, message);
        QTextStream(stdout) << formatted << endl;
    }
}

} // namespace Soro
