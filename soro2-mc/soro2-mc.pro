QT += qml quick opengl quickcontrols2 network webengine

CONFIG += c++11

HEADERS +=

SOURCES += main.cpp

RESOURCES += qml.qrc \
    assets.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

INCLUDEPATH += /opt/ros/kinetic/include/
DEPENDPATH += /opt/ros/kinetic/include/

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
LIBS += -L/opt/ros/kinetic/lib -lparams
