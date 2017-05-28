QT += core network
QT -= gui

CONFIG += c++11 no_keywords

TARGET = soro_science_controller
CONFIG += console
CONFIG -= app_bundle

BUILD_DIR = ../build/soro_science_controller
DESTDIR = ../bin

TEMPLATE = app

SOURCES += main.cpp \
    maincontroller.cpp \
    sciencepackagecontroller.cpp \
    settingsmodel.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    maincontroller.h \
    sciencepackagecontroller.h \
    settingsmodel.h

# Include headers from other subprojects
INCLUDEPATH += $$PWD/..

# Link against soro_core
LIBS += -L../lib -lsoro_core

# Link against qmqtt
LIBS += -L../lib -lqmqtt
