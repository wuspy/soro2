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


QT       += network widgets

TARGET = soromc
TEMPLATE = lib

# NO_KEYWORDS: signal, slot, emit, etc. will not compile. Use Q_SIGNALS, Q_SLOTS, Q_EMIT instead
CONFIG += no_keywords c++11

DEFINES += LIBSOROMC_LIBRARY

BUILD_DIR = ../build/soro_mc
DESTDIR = ../lib

SOURCES += \
    camerasettingsmodel.cpp \
    logger.cpp \
    gstreamerutil.cpp \
    abstractsettingsmodel.cpp

HEADERS +=\
    libsoromc_global.h \
    camerasettingsmodel.h \
    constants.h \
    logger.h \
    armmessage.h \
    gstreamerutil.h \
    abstractsettingsmodel.h

# Link against SDL2
LIBS += -lSDL2

# In case you are curious, the following is a hacky way to link against ROS kinetic
# from a qmake project.
#
# Of course you could just use cmake and catkin, however if you have a compelling
# reason to stick with qmake (like we did) then here you go.

# Include ROS headers
INCLUDEPATH += /opt/ros/kinetic/include/
DEPENDPATH += /opt/ros/kinetic/include/

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
