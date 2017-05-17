QT += core network dbus
QT -= gui

CONFIG += c++11 no_keywords
CONFIG += console
CONFIG -= app_bundle

TARGET = soro_audioserver

BUILD_DIR = ../build/soro_audioserver
DESTDIR = ../bin

TEMPLATE = app

HEADERS += \
    audioserver.h \
    maincontroller.h \
    settingsmodel.h

SOURCES += main.cpp \
    audioserver.cpp \
    maincontroller.cpp \
    settingsmodel.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# Include headers from other subprojects
INCLUDEPATH += $$PWD/..

# Link against soro_core
LIBS += -L../lib -lsoro_core

#Link Qt5GStreamer
LIBS += -lQt5GStreamer-1.0 -lQt5GLib-2.0

# Link against qmqtt
LIBS += -L../lib -lqmqtt
