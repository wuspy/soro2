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

QT += qml quick opengl quickcontrols2 network webengine

CONFIG += no_keywords c++11

TARGET = soro_mc

BUILD_DIR = ../build/soro_mc
DESTDIR = ../bin

INCLUDEPATH += $$PWD/..

HEADERS += \
    drivecontrolsystem.h \
    gamepadcontroller.h \
    maincontroller.h \
    mainwindowcontroller.h \
    qquickgstreamersurface.h \
    connectionstatuscontroller.h \
    settingsmodel.h \
    masterlocator.h

SOURCES += main.cpp \
    gamepadcontroller.cpp \
    maincontroller.cpp \
    mainwindowcontroller.cpp \
    qquickgstreamersurface.cpp \
    drivecontrolsystem.cpp \
    connectionstatuscontroller.cpp \
    settingsmodel.cpp \
    masterlocator.cpp

RESOURCES += qml.qrc \
    assets.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Include ROS headers
INCLUDEPATH += /opt/ros/kinetic/include/
DEPENDPATH += /opt/ros/kinetic/include/

# Link against libsoromc
LIBS += -L../lib -lsoromc

# Link against SDL2
LIBS += -lSDL2

# Link against Qt5Gstreamer
LIBS += -lQt5GStreamer-1.0 -lQt5GLib-2.0 -lQt5GStreamerUtils-1.0 -lQt5GStreamerQuick-1.0

# Link against ROS
LIBS += -L/opt/ros/kinetic/lib -lroslib
LIBS += -L/opt/ros/kinetic/lib -lroscpp
LIBS += -L/opt/ros/kinetic/lib -lroscpp_serialization
LIBS += -L/opt/ros/kinetic/lib -lcpp_common
LIBS += -L/opt/ros/kinetic/lib -lbondcpp
LIBS += -L/opt/ros/kinetic/lib -lxmlrpcpp
LIBS += -L/opt/ros/kinetic/lib -lmessage_filters
LIBS += -L/opt/ros/kinetic/lib -lrosconsole
LIBS += -L/opt/ros/kinetic/lib -lrosconsole_backend_interface
LIBS += -L/opt/ros/kinetic/lib -lrosconsole_bridge
LIBS += -L/opt/ros/kinetic/lib -lrosconsole_log4cxx
LIBS += -L/opt/ros/kinetic/lib -lrostime
LIBS += -L/opt/ros/kinetic/lib -lrospack
