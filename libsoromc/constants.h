#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <QCoreApplication>

#define SORO_SETTINGS_DIR QCoreApplication::applicationDirPath() + "/../config"
#define SORO_MC_MASTER_PROCESS_PATH QCoreApplication::applicationDirPath() + "/soro_mc_master"
#define SORO_MC_SETTINGS_FILE SORO_SETTINGS_DIR + "/mc.conf"
#define SORO_CAMERA_SETTINGS_FILE SORO_SETTINGS_DIR + "/cameras.json"

#define SORO_MC_ROS_MASTER_PORT 5555
#define SORO_MC_FIRST_VIDEO_PORT 5560
#define SORO_MC_LAST_VIDEO_PORT 5599
#define SORO_MC_BROADCAST_PORT 5556

#endif // CONSTANTS_H
