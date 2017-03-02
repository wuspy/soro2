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

    property alias mainVideoSurface: mainVideoSurface
    property alias sideVideoSurface1: sideVideoSurface1
    property alias sideVideoSurface2: sideVideoSurface2

    /*
      The web view that shows the Google Maps overlay
      */
    WebEngineView {
        id: mapWebEngine
        anchors.fill: parent
        url: "qrc:/html/map.html"
    }

    GStreamerSurface {
        id: mainVideoSurface
        anchors.fill: parent;
    }

    DropShadow {
        anchors.fill: sidebar
        source: sidebar

        radius: 32
        samples: 64
    }

    FastBlur {
        id: sidebarBlur
        anchors.fill: sidebar
        source: ShaderEffectSource {
            sourceItem: mainVideoSurface
            sourceRect: Qt.rect(sidebar.x, sidebar.y, sidebar.width, sidebar.height)
        }

        radius: 64
        opacity: 1
    }

    Rectangle {
        id: sidebar
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: 400
        opacity: 1
        color: "#88000000"

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

            Timer {
                id: pingChangeTimer
                repeat: true
                interval: 500
                running: true
                onTriggered: {
                    pingLabel.text = Math.round(Math.random() * 8) + "ms"
                }
            }
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

        Rectangle {
            id: viewSelectionRectangle
            anchors.fill: parent
            anchors.topMargin: 150
            color: "transparent"

            Rectangle {
                id: viewSelection1
                height: width / 1.6
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.rightMargin: 20
                anchors.leftMargin: 20
                anchors.top: parent.top
                anchors.topMargin: 20

                border.color: "#2196F3"
                border.width: 4
                GStreamerSurface {
                    id: sideVideoSurface1
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                    backgroundColor: "red"
                }
                DropShadow {
                    source: viewSelection1Text
                    anchors.fill: viewelection1Text
                    radius: 10
                    samples: 20
                    color: "black"
                }

                Text {
                    id: viewSelection1Text
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 12
                    font.pixelSize: 24
                    font.bold: true
                    color: "white"
                    text: "Camera 1"
                }
            }

            Rectangle {
                id: viewSelection2
                height: width / 1.6
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.rightMargin: 20
                anchors.leftMargin: 20
                anchors.top: viewSelection1.bottom
                anchors.topMargin: 20

                border.color: "white"
                border.width: 4
                GStreamerSurface {
                    id: sideVideoSurface2
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                }
                DropShadow {
                    source: viewSelection2Text
                    anchors.fill: viewSelection2Text
                    radius: 10
                    samples: 20
                    color: "black"
                }

                Text {
                    id: viewSelection2Text
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 12
                    font.pixelSize: 24
                    font.bold: true
                    color: "white"
                    text: "Camera 2"
                }
            }

            Rectangle {
                id: viewSelection3
                height: width / 1.6
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.rightMargin: 20
                anchors.leftMargin: 20
                anchors.top: viewSelection2.bottom
                anchors.topMargin: 20
                border.color: "white"
                border.width: 4
                WebEngineView {
                    id: mapWebEngine2
                    anchors.fill: parent
                    url: "qrc:/html/map.html"
                    anchors.margins: parent.border.width
                }
                DropShadow {
                    source: viewSelection3Text
                    anchors.fill: viewSelection3Text
                    radius: 10
                    samples: 20
                    color: "black"
                }

                Text {
                    id: viewSelection3Text
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 12
                    font.pixelSize: 24
                    font.bold: true
                    color: "white"
                    text: "Map"
                }
            }

        }

        Rectangle {
            id: separator1
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: viewSelectionRectangle.top

            height: 2
            color: "white"
            opacity: 0.2
        }
    }
}
