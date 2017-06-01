QT += core network
QT -= gui

CONFIG += c++11 no_keywords

TARGET = soro_arm_controller
CONFIG += console
CONFIG -= app_bundle

BUILD_DIR = ../build/soro_arm_controller
DESTDIR = ../bin

TEMPLATE = app

SOURCES += main.cpp \
    maincontroller.cpp \
    armcontroller.cpp \
    settingsmodel.cpp

DEFINES += QT_DEPRECATED_WARNINGS

HEADERS += \
    maincontroller.h \
    armcontroller.h \
    settingsmodel.h

# Include headers from other subprojects
INCLUDEPATH += $$PWD/..

# Link against soro_core
LIBS += -L../lib -lsoro_core

# Link against qmqtt
LIBS += -L../lib -lqmqtt
