#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <QCoreApplication>

#define SORO_MC_SETTINGS_DIR QCoreApplication::applicationDirPath() + "/../config"

#define SORO_ROVER_VIDEO_STREAM_PROCESS_PATH QCoreApplication::applicationDirPath() + "/soro_videostreamer"
#define SORO_ROVER_AUDIO_STREAM_PROCESS_PATH QCoreApplication::applicationDirPath() + "/soro_audiostreamer"

#define SORO_DBUS_SERVICE_NAME_ROOT "edu.ou.soonerrover"
#define SORO_DBUS_VIDEO_PARENT_SERVICE_NAME "edu.ou.soonerrover.video_parent"
#define SORO_DBUS_VIDEO_CHILD_SERVICE_NAME(id) "edu.ou.soonerrover.video_child_" + id
#define SORO_DBUS_AUDIO_PARENT_SERVICE_NAME "edu.ou.soonerrover.audio_parent"
#define SORO_DBUS_AUDIO_CHILD_SERVICE_NAME "edu.ou.soonerrover.audio_child"

#define SORO_NET_MC_BROADCAST_PORT          5552
#define SORO_NET_GIMBAL_SYSTEM_PORT         5553
#define SORO_NET_ARM_SYSTEM_PORT            5554
#define SORO_NET_DRIVE_SYSTEM_PORT          5556
#define SORO_NET_MQTT_BROKER_PORT           1883
#define SORO_NET_MASTER_ARM_PORT            5558
#define SORO_NET_AUDIO_PORT                 5559
#define SORO_NET_FIRST_VIDEO_PORT           5560
#define SORO_NET_LAST_VIDEO_PORT            5650
#define SORO_NET_MC_AUDIO_PORT              5659
#define SORO_NET_MC_FIRST_VIDEO_PORT        5660
#define SORO_NET_MC_LAST_VIDEO_PORT         5750

#define NOTIFICATION_TYPE_ERROR     0
#define NOTIFICATION_TYPE_WARNING   1
#define NOTIFICATION_TYPE_INFO      2

#endif // CONSTANTS_H
