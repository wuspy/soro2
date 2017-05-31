QT += qml quick opengl quickcontrols2 network webengine

# NO_KEYWORDS: signal, slot, emit, etc. will not compile. Use Q_SIGNALS, Q_SLOTS, Q_EMIT instead
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
    connectionstatuscontroller.h \
    settingsmodel.h \
    gstreamerpipelinewatch.h \
    qmlgstreamerpainteditem.h \
    qmlgstreamerglitem.h \
    bindssettingsmodel.h \
    videoclient.h \
    mapviewimpl.h \
    sciencearmcontrolsystem.h \
    armcontrolsystem.h \
    sciencecameracontrolsystem.h \
    audioclient.h \
    pitchrollview.h

SOURCES += main.cpp \
    gamepadcontroller.cpp \
    maincontroller.cpp \
    mainwindowcontroller.cpp \
    drivecontrolsystem.cpp \
    connectionstatuscontroller.cpp \
    settingsmodel.cpp \
    gstreamerpipelinewatch.cpp \
    qmlgstreamerpainteditem.cpp \
    qmlgstreamerglitem.cpp \
    bindssettingsmodel.cpp \
    videoclient.cpp \
<<<<<<< HEAD
    mapviewimpl.cpp
=======
    sciencearmcontrolsystem.cpp \
    armcontrolsystem.cpp \
    sciencecameracontrolsystem.cpp \
    audioclient.cpp \
    pitchrollview.cpp
>>>>>>> 6bb7c64db6f8dd79386f2e386eba9639054d1332

RESOURCES += qml.qrc \
    assets.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Link against soro_core
LIBS += -L../lib -lsoro_core

# Link against qmqtt
LIBS += -L../lib -lqmqtt

# Link against SDL2
LIBS += -lSDL2

# Link against Qt5Gstreamer
LIBS += -lQt5GStreamer-1.0 -lQt5GLib-2.0 -lQt5GStreamerUtils-1.0 -lQt5GStreamerQuick-1.0
