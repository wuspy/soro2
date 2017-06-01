QT += core network
QT -= gui

CONFIG += c++11 no_keywords

TARGET = soro_drive_controller
CONFIG += console
CONFIG -= app_bundle

BUILD_DIR = ../build/soro_drive_controller
DESTDIR = ../bin

TEMPLATE = app

SOURCES += main.cpp \
    settingsmodel.cpp \
    maincontroller.cpp \
    drivecontroller.cpp

DEFINES += QT_DEPRECATED_WARNINGS

# Include headers from other subprojects
INCLUDEPATH += $$PWD/..

# Link against soro_core
LIBS += -L../lib -lsoro_core

# Link against qmqtt
LIBS += -L../lib -lqmqtt

HEADERS += \
    settingsmodel.h \
    maincontroller.h \
    drivecontroller.h
