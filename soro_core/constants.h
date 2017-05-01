#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <QCoreApplication>

#define SORO_MC_SETTINGS_DIR QCoreApplication::applicationDirPath() + "/../config"
#define SORO_MC_MASTER_PROCESS_PATH QCoreApplication::applicationDirPath() + "/soro_mc_master"
#define SORO_ROVER_STREAM_PROCESS_PATH QCoreApplication::applicationDirPath() + "/soro_streamer"

#define SORO_DBUS_SERVICE_NAME "edu.ou.soonerrover"
#define SORO_DBUS_VIDEO_PARENT_OBJECT_NAME "/videoParent"
#define SORO_DBUS_VIDEO_CHILD_OBJECT_NAME(pid) "/videoChild_" + pid
#define SORO_DBUS_AUDIO_PARENT_OBJECT_NAME "/audioParent"
#define SORO_DBUS_AUIDO_CHILD_OBJECT_NAME(pid) "/audioChild_" + pid

#define SORO_NET_MC_BROADCAST_PORT              5557
#define SORO_NET_MASTER_ARM_PORT                5558
#define SORO_NET_AUDIO_PORT                     5559
#define SORO_NET_FIRST_VIDEO_PORT               5560

#define NOTIFICATION_TYPE_ERROR     0
#define NOTIFICATION_TYPE_WARNING   1
#define NOTIFICATION_TYPE_INFO      2

#endif // CONSTANTS_H
