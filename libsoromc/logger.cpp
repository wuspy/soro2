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

#include <ros/ros.h>
#include <QTextStream>

namespace Soro {

Logger *Logger::_self = nullptr;

Logger::Logger() { }

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

void Logger::log(LogLevel level, QString tag, QString message) {
    if (ros::isInitialized()) {
        const char* formatted = QString("[%1] %2").arg(tag, message).toLocal8Bit().constData();
        switch (level) {
        case LogLevelDebug:
            ROS_DEBUG(formatted);
            break;
        case LogLevelInfo:
            ROS_INFO(formatted);
            break;
        case LogLevelWarning:
            ROS_WARN(formatted);
            break;
        case LogLevelError:
            ROS_ERROR(formatted);
            break;
        }
    }
    else {
        QString levelString;
        switch (level) {
        case LogLevelDebug:
            levelString = "DEBUG";
            break;
        case LogLevelInfo:
            levelString = "INFO";
            break;
        case LogLevelWarning:
            levelString = "WARN";
            break;
        case LogLevelError:
            levelString = "ERROR";
            break;
        }
        QString formatted = QString("[%1] [%2] %3").arg(levelString, tag, message);
        QTextStream(stdout) << formatted << endl;
    }
}

} // namespace Soro
