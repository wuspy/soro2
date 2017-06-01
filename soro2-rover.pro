TEMPLATE = subdirs

SUBDIRS =\
    soro_core \
    soro_videostreamer \
    soro_audiostreamer \
    soro_audioserver \
    soro_videoserver \
    soro_arm_controller \
    soro_science_controller \
    soro_drive_controller \
    qmqtt

soro_core.depends = qmqtt
soro_arm_controller.depends = soro_core qmqtt
soro_science_controller.depends = soro_core qmqtt
soro_videostreamer.depends = soro_core qmqtt
soro_audiostreamer.depends = soro_core qmqtt
soro_audioserver.depends = soro_core qmqtt
soro_videoserver.depends = soro_core qmqtt
soro_drive_controller.depends = soro_core qmqtt
