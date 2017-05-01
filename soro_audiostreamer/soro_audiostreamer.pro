QT += core network dbus

CONFIG += c++11 no_keywords
CONFIG += console
CONFIG -= app_bundle

TARGET = soro_audiostreamer

BUILD_DIR = ../build/soro_audiostreamer
DESTDIR = ../bin

TEMPLATE = app

HEADERS += audiostreamer.h

SOURCES += main.cpp \
    audiostreamer.cpp

# Include headers from other subprojects
INCLUDEPATH += $$PWD/..

#Link Qt5GStreamer
LIBS += -lQt5GStreamer-1.0 -lQt5GLib-2.0

# Link against soro_core
LIBS += -L../lib -lsoro_core

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
