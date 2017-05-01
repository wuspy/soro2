## Copyright 2017 The University of Oklahoma.
##
## Licensed under the Apache License, Version 2.0 (the "License");
## you may not use this file except in compliance with the License.
## You may obtain a copy of the License at
##
##     http://www.apache.org/licenses/LICENSE-2.0
##
## Unless required by applicable law or agreed to in writing, software
## distributed under the License is distributed on an "AS IS" BASIS,
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
## See the License for the specific language governing permissions and
## limitations under the License.

TEMPLATE = subdirs

SUBDIRS =\
    soro_mc \
    soro_mc_master \
    soro_core \
    soro_videostreamer \
    soro_audiostreamer \
    soro_audioserver \
    soro_videoserver

soro_mc.depends = soro_core
soro_mc_master.depends = soro_core
soro_videostreamer.depends = soro_core
soro_audiostreamer.depends = soro_core
soro_audioserver.depends = soro_core
soro_videoserver.depends = soro_core
