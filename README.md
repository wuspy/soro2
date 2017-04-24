# soro2-mc

This is the repository for the mission control software for the 2016-1017 Sooner Rover Team.

## Dependencies

### Qt 5.8

Qt is the main development platform for this software. We target version 5.8, however other versions should work as long as they support Qt Quick Controls 2 and Qt Webengine.

It is recommended to use the Qt development environment from [the Qt website](https://www.qt.io/), rather than your distributions builtin Qt environment.

### ROS Kinetic

ROS is used as the communication library for mission control and the rover. You will need to be using a distribution that supports ROS kinetic, such as Ubuntu 16.04 or Arch.

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
