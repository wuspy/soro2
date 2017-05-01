#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <QCoreApplication>

#define SORO_MC_SETTINGS_DIR QCoreApplication::applicationDirPath() + "/../config"
#define SORO_MC_MASTER_PROCESS_PATH QCoreApplication::applicationDirPath() + "/soro_mc_master"
#define SORO_ROVER_STREAM_PROCESS_PATH QCoreApplication::applicationDirPath() + "/soro_streamer"

#define SORO_NET_ROS_MASTER_PORT        5555
#define SORO_NET_MASTER_BROADCAST_PORT  5556
#define SORO_NET_MASTER_ARM_PORT        5558
#define SORO_NET_AUDIO_PORT             5559
#define SORO_NET_FIRST_VIDEO_PORT       5560
#define SORO_NET_LAST_VIDEO_PORT        5599

#define NOTIFICATION_TYPE_ERROR     0
#define NOTIFICATION_TYPE_WARNING   1
#define NOTIFICATION_TYPE_INFO      2

#endif // CONSTANTS_H
