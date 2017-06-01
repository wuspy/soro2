#ifndef CONSTANTS_H
#define CONSTANTS_H

#ifdef QT_CORE_LIB

#include <QCoreApplication>

#define SORO_MC_SETTINGS_DIR QCoreApplication::applicationDirPath() + "/../config"

#define SORO_ROVER_VIDEO_STREAM_PROCESS_PATH QCoreApplication::applicationDirPath() + "/soro_videostreamer"
#define SORO_ROVER_AUDIO_STREAM_PROCESS_PATH QCoreApplication::applicationDirPath() + "/soro_audiostreamer"

#endif // QT_CORE_LIB

#define SORO_DBUS_SERVICE_NAME_ROOT "edu.ou.soonerrover"
#define SORO_DBUS_VIDEO_PARENT_SERVICE_NAME "edu.ou.soonerrover.video_parent"
#define SORO_DBUS_VIDEO_CHILD_SERVICE_NAME(id) "edu.ou.soonerrover.video_child_" + id
#define SORO_DBUS_AUDIO_PARENT_SERVICE_NAME "edu.ou.soonerrover.audio_parent"
#define SORO_DBUS_AUDIO_CHILD_SERVICE_NAME "edu.ou.soonerrover.audio_child"

#define SORO_NET_MQTT_BROKER_PORT           1883

#define SORO_HEADER_MASTER_ARM_MSG          'm'
#define SORO_HEADER_SCIENCE_MASTER_ARM_MSG  'a'
#define SORO_HEADER_SLAVE_ARM_MSG           's'
#define SORO_HEADER_SCIENCE_PACKAGE_MSG     'b'
#define SORO_HEADER_SCIENCE_CONTROLLER_MSG  'c'
#define SORO_HEADER_DRIVE_MSG               'd'
#define SORO_HEADER_DRIVE_HEARTBEAT_MSG     'e'

#define SORO_HEADER_ARM_KILL                '0'
#define SORO_HEADER_ARM_YAW                 '1'
#define SORO_HEADER_ARM_SHOULDER            '2'
#define SORO_HEADER_ARM_ELBOW               '3'
#define SORO_HEADER_ARM_WRIST               '4'
#define SORO_HEADER_ARM_ROLL                '5'
#define SORO_HEADER_ARM_CLAW                '6'
#define SORO_HEADER_ARM_RESET               '7'

#define SORO_HEADER_SCIENCE_ARM_STOW        'A'
#define SORO_HEADER_SCIENCE_ARM_YAW         'B'
#define SORO_HEADER_SCIENCE_ARM_SHOULDER    'C'
#define SORO_HEADER_SCIENCE_ARM_ELBOW       'D'
#define SORO_HEADER_SCIENCE_ARM_WRIST       'E'
#define SORO_HEADER_SCIENCE_ARM_CLAW        'F'
#define SORO_HEADER_SCIENCE_SPEC            'G'
#define SORO_HEADER_SCIENCE_ATMOSPHERE      'H'
#define SORO_HEADER_SCIENCE_GEIGER          'I'
#define SORO_HEADER_SCIENCE_PROBE_ON        'J'
#define SORO_HEADER_SCIENCE_PROBE_OFF       'K'
#define SORO_HEADER_SCIENCE_PROBE_POSITION  'L'
#define SORO_HEADER_SCIENCE_DRILL_POSITION  'M'
#define SORO_HEADER_SCIENCE_SPEC_ON         'N'
#define SORO_HEADER_SCIENCE_SPEC_OFF        'O'
#define SORO_HEADER_SCIENCE_ATMOSPHERE_ON   'P'
#define SORO_HEADER_SCIENCE_ATMOSPHERE_OFF  'Q'
#define SORO_HEADER_SCIENCE_GEIGER_ON       'R'
#define SORO_HEADER_SCIENCE_GEIGER_OFF      'S'
#define SORO_HEADER_SCIENCE_DRILL_ON        'T'
#define SORO_HEADER_SCIENCE_DRILL_OFF       'U'

#define SORO_NET_SLAVE_ARM_PORT             5554
#define SORO_NET_SCIENCE_SYSTEM_PORT        5555
#define SORO_NET_DRIVE_SYSTEM_PORT          5556
#define SORO_NET_SCIENCE_MASTER_ARM_PORT    5557
#define SORO_NET_MASTER_ARM_PORT            5558
#define SORO_NET_AUDIO_PORT                 5559
#define SORO_NET_FIRST_VIDEO_PORT           5560
#define SORO_NET_LAST_VIDEO_PORT            5650
#define SORO_NET_MC_AUDIO_PORT              5659
#define SORO_NET_MC_FIRST_VIDEO_PORT        5660
#define SORO_NET_MC_LAST_VIDEO_PORT         5750

#endif // CONSTANTS_H
