# soro2

This is the repository for all of the Qt-based software for the 2016-2017 Sooner Rover Team. It includes:
 * All Mission Control components
 * The Rover video server
 
Additional software, written in other languages, is also needed to complete this software suite. Links to these projects will be added here shortly.

## Dependencies

### Qt 5.8

Qt is the main development platform for this software. We target version 5.8, however other versions should work as long as they support Qt Quick Controls 2 and Qt Webengine.

It is recommended to use the Qt development environment from [the Qt website](https://www.qt.io/), rather than your distributions builtin Qt environment.

### QMQTT

QMQTT is a Qt implementation for the MQTT protocol. It is included in this repository for convenience.

### SDL2

SDL is used for gamepad input and haptic feedback. Search your distribution's repository for libsdl2 development packages.

### GStreamer 1.0

GStreamear is the core of our media stack. It is recommended you install all GStreamer libraries and tools.

### Qt5GStreamer

This library provides Qt-style bindings for GStreamer. Search your distribution's repository for libqt5gstreamer development packages.

## Compiling

This software is only developed for Linux, and has been tested on Ubuntu 16.04 and Arch.

Clone this repository into a directory, and open the main soro2-mc.pro project file in QtCreator. If all necessary dependencies are installed correctly, it should be able to compile without any problems.

You will need to provide several configuration files to the program in the {build_dir}/config directory. You can find examples for these files in the config directory of this repository.

## License

Content in this repository is licensed on a per-file basis. Refer to the license header in each file.
