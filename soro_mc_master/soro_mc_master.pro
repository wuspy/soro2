QT += network qml quick widgets

# NO_KEYWORDS: signal, slot, emit, etc. will not compile. Use Q_SIGNALS, Q_SLOTS, Q_EMIT instead
CONFIG += no_keywords c++11

TARGET = soro_mc_master

BUILD_DIR = ../build/soro_mc_master
DESTDIR = ../bin

TEMPLATE = app

INCLUDEPATH += $$PWD/..

SOURCES += main.cpp \
    mainwindowcontroller.cpp \
    maincontroller.cpp \
    masterconnectionstatuscontroller.cpp \
    settingsmodel.cpp \
    mastervideocontroller.cpp \
    masteraudiocontroller.cpp

HEADERS += \
    mainwindowcontroller.h \
    maincontroller.h \
    masterconnectionstatuscontroller.h \
    settingsmodel.h \
    mastervideocontroller.h \
    masteraudiocontroller.h

RESOURCES += \
    qml.qrc \
    assets.qrc

# Link against soro_core
LIBS += -L../lib -lsoro_core

# Link against qmqtt
LIBS += -L../lib -lqmqtt
