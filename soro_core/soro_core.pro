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
    notificationmessage.cpp \
    spectrometermessage.cpp \
    atmospheresensormessage.cpp \
    geigermessage.cpp \
    compassmessage.cpp \
    gpsmessage.cpp \
    drivepathmessage.cpp \
    switchmessage.cpp \
    sciencecameragimbalmessage.cpp

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
    abstractmessage.h \
    notificationmessage.h \
    spectrometermessage.h \
    atmospheresensormessage.h \
    geigermessage.h \
    compassmessage.h \
    gpsmessage.h \
    drivepathmessage.h \
    serialize.h \
    switchmessage.h \
    sciencecameragimbalmessage.h

# Link against qmqtt
LIBS += -L../lib -lqmqtt

