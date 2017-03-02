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
    property alias mainVideoSurface: mainVideoSurface

    /*
      State of the sidebar, can be either 'hidden' or 'visible'
      */
    property alias sidebarState: sidebar.state

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
        id: mainVideoSurface
        anchors.fill: parent;
        visible: false
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
            sourceItem: mainVideoSurface.visible ? mainVideoSurface : mapWebEngine
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
            source: "qrc:/icons/ic_check_circle_white_48px.svg"
        }

        ColorOverlay {
            anchors.fill: connectionStatusImage
            source: connectionStatusImage
            color: "#4CAF50"
        }

        Label {
            id: connectionStatusLabel
            font.pixelSize: 48
            anchors.right: parent.right
            anchors.rightMargin: 10
            anchors.leftMargin: 10
            anchors.left: connectionStatusImage.right
            anchors.verticalCenter: connectionStatusImage.verticalCenter
            text: "Connected"
            color: "#4CAF50"
        }

        Label {
            id: pingLabel
            font.pixelSize: 48
            anchors.top: connectionStatusImage.bottom
            anchors.right: parent.right
            anchors.rightMargin: 10
            anchors.left: parent.horizontalCenter
            text: "18ms"
            horizontalAlignment: Text.AlignHCenter
            color: "white"
        }

        Label {
            id: bitrateLabel
            text: qsTr("▲ <b>13</b>  b/s<br>▼ <b>9</b> Mb/s")
            anchors.top: connectionStatusImage.bottom
            anchors.left: connectionStatusImage.left
            anchors.right: parent.horizontalCenter
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: 24
            color: "white"
        }
    }

    /*
      Drop shadow for the mini connection status image
      */
    DropShadow {
        source: miniConnectionStatusImage
        anchors.fill: miniConnectionStatusImage
        radius: 20
        samples: 40
        color: "#66000000"
        visible: sidebar.state == "hidden"
    }

    /*
      Status image that is shown when the main sidebar is hidden
      */
    Image {
        id: miniConnectionStatusImage
        width: 96
        height: 96
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.leftMargin: 10
        anchors.topMargin: 10
        visible: false
        sourceSize: Qt.size(width, height)
        source: "qrc:/icons/ic_check_circle_white_48px.svg"
    }

    /*
      Color overlay to colorize the mini connection status image
      */
    ColorOverlay {
        anchors.fill: miniConnectionStatusImage
        source: miniConnectionStatusImage
        color: "#4CAF50"
        visible: sidebar.state == "hidden"
    }

    /*
      Drop shadow for the mini ping label
      */
    DropShadow {
        source: miniPingLabel
        anchors.fill: miniPingLabel
        radius: 20
        samples: 40
        color: "#66000000"
        visible: sidebar.state == "hidden"
    }

    /*
      Ping label that is shown when the main sidebar is hidden
      */
    Label {
        id: miniPingLabel
        font.pixelSize: 64
        anchors.rightMargin: 10
        anchors.leftMargin: 10
        anchors.left: miniConnectionStatusImage.right
        anchors.verticalCenter: miniConnectionStatusImage.verticalCenter
        text: pingLabel.text
        visible: sidebar.state == "hidden"
        color: "#4CAF50"
    }

}
