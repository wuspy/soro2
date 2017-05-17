## Copyright 2017 The University of Oklahoma.
##
## Licensed under the Apache License, Version 2.0 (the "License");
## you may not use this file except in compliance with the License.
## You may obtain a copy of the License at
##
##     http://www.apache.org/licenses/LICENSE-2.0
##
## Unless required by applicable law or agreed to in writing, software
## distributed under the License is distributed on an "AS IS" BASIS,
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
## See the License for the specific language governing permissions and
## limitations under the License.


QT       += network widgets dbus

TARGET = soro_core
TEMPLATE = lib

# NO_KEYWORDS: signal, slot, emit, etc. will not compile. Use Q_SIGNALS, Q_SLOTS, Q_EMIT instead
CONFIG += no_keywords c++11

DEFINES += SORO_CORE_LIBRARY

BUILD_DIR = ../build/soro_core
DESTDIR = ../lib

INCLUDEPATH += $$PWD/..

SOURCES += \
    camerasettingsmodel.cpp \
    logger.cpp \
    gstreamerutil.cpp \
    abstractsettingsmodel.cpp \
    mediaprofilesettingsmodel.cpp \
    videomessage.cpp \
    videostatemessage.cpp \
    audiomessage.cpp \
    drivemessage.cpp \
    armmessage.cpp \
    latencymessage.cpp \
    dataratemessage.cpp \
    pingmessage.cpp \
    addmediabouncemessage.cpp \
    coresettingsmodel.cpp \
    notificationmessage.cpp

HEADERS +=\
    soro_core_global.h \
    camerasettingsmodel.h \
    constants.h \
    logger.h \
    gstreamerutil.h \
    abstractsettingsmodel.h \
    mediaprofilesettingsmodel.h \
    videomessage.h \
    videostatemessage.h \
    audiomessage.h \
    drivemessage.h \
    armmessage.h \
    latencymessage.h \
    dataratemessage.h \
    pingmessage.h \
    addmediabouncemessage.h \
    coresettingsmodel.h \
    abstractmessage.h \
    notificationmessage.h

# Link against qmqtt
LIBS += -L../lib -lqmqtt

