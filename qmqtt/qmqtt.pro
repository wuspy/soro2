QT += core network
QT -= gui

TARGET = qmqtt
TEMPLATE = lib

DEFINES += QT_NO_CAST_TO_ASCII QT_NO_CAST_FROM_ASCII
    
CONFIG += c++11

DEFINES += QMQTT_LIBRARY

BUILD_DIR = ../build/qmqtt
DESTDIR = ../lib

HEADERS += qmqtt_global.h \
    qmqtt.h \
    qmqtt_client_p.h \
    qmqtt_message_p.h \
    qmqtt_network_p.h \
    qmqtt_socket_p.h \
    qmqtt_ssl_network_p.h \
    qmqtt_ssl_socket_p.h \
    qmqtt_timer_p.h \
    qmqtt_client.h \
    qmqtt_frame.h \
    qmqtt_message.h \
    qmqtt_routesubscription.h \
    qmqtt_routedmessage.h \
    qmqtt_router.h \
    qmqtt_networkinterface.h \
    qmqtt_socketinterface.h \
    qmqtt_timerinterface.h
    
SOURCES += \
    qmqtt_client_p.cpp \
    qmqtt_client.cpp \
    qmqtt_frame.cpp \
    qmqtt_message.cpp \
    qmqtt_network.cpp \
    qmqtt_ssl_network.cpp \
    qmqtt_routesubscription.cpp \
    qmqtt_routedmessage.cpp \
    qmqtt_router.cpp \
    qmqtt_message_p.cpp \
    qmqtt_socket.cpp \
    qmqtt_ssl_socket.cpp \
    qmqtt_timer.cpp
