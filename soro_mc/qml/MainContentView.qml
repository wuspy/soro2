/*
 * Copyright 2017 Jacob Jordan <doublejinitials@ou.edu>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

import QtQuick 2.7
import QtWebEngine 1.4
import Soro 1.0

Item {
    id: mainContentView

    property alias mapView: mapView
    property alias mapImage: mapView.image
    property alias mapViewImpl: mapView.impl
    property alias spectrometerView: spectrometer
    property variant videoSurfaces: []

    property int videoCount: 0
    readonly property int viewCount: videoCount + 2
    readonly property int mapIndex: videoCount
    readonly property int spectrometerIndex: videoCount + 1

    property int activeViewIndex: -1

    onActiveViewIndexChanged: {
        for (var i = 0; i < videoSurfaces.length; ++i) {
            videoSurfaces[i].z = i == activeViewIndex ? 1 : 0
            videoSurfaces[i].enabled = i == activeViewIndex
        }
        mapView.z = activeViewIndex == mapIndex ? 1 : 0
        mapView.enabled = activeViewIndex == mapIndex
        spectrometer.z = activeViewIndex == spectrometerIndex ? 1 : 0
        spectrometer.enabled = activeViewIndex == spectrometerIndex
    }

    onVideoCountChanged: {
        while (videoSurfaces.length > 0) {
            videoSurfaces.pop().destroy()
        }

        for (var i = 0; i < videoCount; ++i) {
            var surface = Qt.createQmlObject("import QtQuick 2.7; import Soro 1.0; GStreamerSurface { anchors.fill: parent; z: 0; enabled: false; focus: false; }", mainContentView, "")
            videoSurfaces.push(surface)
        }
        activeViewIndex = 0
    }

    /*
      The web view that shows the Google Maps overlay
      */
    MapView {
        id: mapView
        anchors.fill: parent
        z: 0
        enabled: false
    }

    SpectrometerView {
        id: spectrometer
        anchors.fill: parent
        z: 0
        enabled: false
        focus: false
    }
}
