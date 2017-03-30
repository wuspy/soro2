/*
 * Copyright 2017 The University of Oklahoma.
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
import QtQuick.Controls 2.0
import QtQuick.Controls.Material 2.0
import QtQuick.Layouts 1.0
import QtGraphicalEffects 1.0
import QtWebEngine 1.4
import Soro 1.0

ApplicationWindow {
    visible: true
    width: 800
    height: 600
    title: "Mission Control"

    /*
      Alias properties for QML controls
      */
    property alias mainGstreamerSurface: mainGstreamerSurface

    /*
      State of the sidebar, can be either 'hidden' or 'visible'
      */
    property alias sidebarState: sidebar.state

    property int latency: 0
    onLatencyChanged: {
        pingLabel.text = latency + "ms"
    }

    property int bitrateDown: 0
    property int bitrateUp: 0

    onBitrateDownChanged: {
        bitrateLabel.text = "▲ <b>" + bitrateUp + "</b>  b/s<br>▼ <b>" + bitrateDown + "</b> Mb/s"
    }

    onBitrateUpChanged: {
        bitrateLabel.text = "▲ <b>" + bitrateUp + "</b>  b/s<br>▼ <b>" + bitrateDown + "</b> Mb/s"
    }

    property alias selectedViewIndex: sidebarViewSelector.selectedIndex
    onSelectedViewIndexChanged: {
        if (selectedViewIndex == cameraCount) {
            mainGstreamerSurface.visible = false
            mapWebEngine.enabled = true
        }
        else {
            mainGstreamerSurface.visible = true
            mapWebEngine.enabled = false
        }
    }

    property alias cameraCount: sidebarViewSelector.cameraCount
    property alias camera0Name: sidebarViewSelector.camera0Name
    property alias camera1Name: sidebarViewSelector.camera1Name
    property alias camera2Name: sidebarViewSelector.camera2Name
    property alias camera3Name: sidebarViewSelector.camera3Name
    property alias camera4Name: sidebarViewSelector.camera4Name
    property alias camera5Name: sidebarViewSelector.camera5Name
    property alias camera6Name: sidebarViewSelector.camera6Name
    property alias camera7Name: sidebarViewSelector.camera7Name
    property alias camera8Name: sidebarViewSelector.camera8Name
    property alias cameraThumbnail0GstreamerSurface: sidebarViewSelector.cameraThumbnail0GstreamerSurface
    property alias cameraThumbnail1GstreamerSurface: sidebarViewSelector.cameraThumbnail1GstreamerSurface
    property alias cameraThumbnail2GstreamerSurface: sidebarViewSelector.cameraThumbnail2GstreamerSurface
    property alias cameraThumbnail3GstreamerSurface: sidebarViewSelector.cameraThumbnail3GstreamerSurface
    property alias cameraThumbnail4GstreamerSurface: sidebarViewSelector.cameraThumbnail4GstreamerSurface
    property alias cameraThumbnail5GstreamerSurface: sidebarViewSelector.cameraThumbnail5GstreamerSurface
    property alias cameraThumbnail6GstreamerSurface: sidebarViewSelector.cameraThumbnail6GstreamerSurface
    property alias cameraThumbnail7GstreamerSurface: sidebarViewSelector.cameraThumbnail7GstreamerSurface
    property alias cameraThumbnail8GstreamerSurface: sidebarViewSelector.cameraThumbnail8GstreamerSurface

    property color theme_yellow: "#FBC02D"
    property color theme_red: "#d32f2f"
    property color theme_green: "#388E3C"

    /*
      Fullscreen state of the application, boolean
      */
    property bool fullscreen: false;
    onFullscreenChanged: {
        if (fullscreen) {
            showFullScreen()
        }
        else {
            showNormal()
        }
    }

    /*
      The web view that shows the Google Maps overlay
      */
    WebEngineView {
        id: mapWebEngine
        anchors.fill: parent
        url: "qrc:/html/map.html"
    }

    /*
      The main video surface to display fullscreen video
      */
    GStreamerSurface {
        id: mainGstreamerSurface
        anchors.fill: parent;
    }

    /*
      Drop shadow for the mini connection status image
      */
    DropShadow {
        source: miniConnectionStatusImageColorOverlay
        anchors.fill: miniConnectionStatusImageColorOverlay
        radius: 20
        samples: 40
        visible: miniConnectionStatusImageColorOverlay.visible
    }

    /*
      Color overlay to colorize the mini connection status image
      */
    ColorOverlay {
        id: miniConnectionStatusImageColorOverlay
        anchors.fill: miniConnectionStatusImage
        source: miniConnectionStatusImage
        color: theme_yellow
        visible: sidebar.state == "hidden"
    }

    /*
      Status image that is shown when the main sidebar is hidden
      */
    Image {
        id: miniConnectionStatusImage
        width: 96
        height: 96
        anchors.top: parent.toptheme_yellow
        anchors.left: parent.left
        anchors.leftMargin: 10
        anchors.topMargin: 10
        visible: false
        sourceSize: Qt.size(width, height)
        source: "qrc:/icons/ic_error_white_48px.svg"
    }

    /*
      Drop shadow for the sidebar
      */
    DropShadow {
        anchors.fill: sidebar
        source: sidebar

        radius: 32
        samples: 64
    }

    /*
      Blur behind the sidebar
      */
    FastBlur {
        id: sidebarBlur
        anchors.fill: sidebar
        source: ShaderEffectSource {
            sourceItem: mainGstreamerSurface.visible ? mainGstreamerSurface : mapWebEngine
            sourceRect: Qt.rect(sidebar.x, sidebar.y, sidebar.width, sidebar.height)
        }

        radius: 64
        opacity: 1
    }

    /*
      The main sidebar
      */
    Rectangle {
        id: sidebar
        state: "visible"
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: 400
        opacity: 1
        color: "#88000000"

        states: [
            State {
                name: "visible"
                PropertyChanges {
                    target: sidebar
                    anchors.leftMargin: 0
                    opacity: 1
                }
            },
            State {
                name: "hidden"
                PropertyChanges {
                    target: sidebar
                    anchors.leftMargin: -width
                    opacity: 0
                }
            }
        ]

        transitions: [
            Transition {
                from: "visible"
                to: "hidden"
                PropertyAnimation {
                    properties: "anchors.leftMargin"
                    duration: 400
                    easing: Easing.InOutQuad
                }
                PropertyAnimation {
                    properties: "opacity"
                    duration: 400
                    easing: Easing.InOutQuad
                }
            },
            Transition {
                from: "hidden"
                to: "visible"
                PropertyAnimation {
                    properties: "anchors.leftMargin"
                    duration: 200
                    easing: Easing.OutInQuad
                }
                PropertyAnimation {
                    properties: "opacity"
                    duration: 300
                    easing: Easing.OutInQuad
                }
            }
        ]

        Image {
            id: connectionStatusImage
            width: 64
            height: 64
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.leftMargin: 10
            anchors.topMargin: 10
            visible: false
            sourceSize: Qt.size(width, height)
            source: "qrc:/icons/ic_error_white_48px.svg"
        }

        ColorOverlay {
            anchors.fill: connectionStatusImage
            source: connectionStatusImage
            color: theme_yellow
        }

        Label {
            id: connectionStatusLabel
            font.pixelSize: 48
            anchors.right: parent.right
            anchors.rightMargin: 10
            anchors.leftMargin: 10
            anchors.left: connectionStatusImage.right
            anchors.verticalCenter: connectionStatusImage.verticalCenter
            text: "Connecting..."
            color: theme_yellow
        }

        Label {
            id: pingLabel
            font.pixelSize: 48
            anchors.top: connectionStatusImage.bottom
            anchors.right: parent.right
            anchors.rightMargin: 10
            anchors.left: parent.horizontalCenter
            text: "---"
            horizontalAlignment: Text.AlignHCenter
            color: "white"
        }

        Label {
            id: bitrateLabel
            text: "▲ <b>0</b>  b/s<br>▼ <b>0</b> b/s"
            anchors.top: connectionStatusImage.bottom
            anchors.left: connectionStatusImage.left
            anchors.right: parent.horizontalCenter
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: 24
            color: "white"
        }

        ViewSelector {
            id: sidebarViewSelector
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: bitrateLabel.bottom
            mapThumbnailViewSource: mapWebEngine
            itemMargins: parent.width / 20
            anchors.leftMargin: itemMargins
            anchors.rightMargin: itemMargins
        }
    }

    NotificationHost {
        anchors.horizontalCenter: parent.horizontalCenter
    }
}
