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
