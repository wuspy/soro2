TEMPLATE = subdirs

SUBDIRS =\
    soro_mc \
    soro_mc_master \
    soro_core \
    soro_videostreamer \
    soro_audiostreamer \
    soro_audioserver \
    soro_videoserver \
    qmqtt

soro_core.depends = qmqtt
soro_mc.depends = soro_core qmqtt
soro_mc_master.depends = soro_core qmqtt
soro_videostreamer.depends = soro_core qmqtt
soro_audiostreamer.depends = soro_core qmqtt
soro_audioserver.depends = soro_core qmqtt
soro_videoserver.depends = soro_core qmqtt
