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
    property alias view1Selected: viewSelection1.selected
    property alias view2Selected: viewSelection2.selected
    property alias view3Selected: viewSelection3.selected

    property alias sidebarState: sidebar.state

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
            sourceItem: mainVideoSurface.visible ? mainVideoSurface : mapWebEngine
            sourceRect: Qt.rect(sidebar.x, sidebar.y, sidebar.width, sidebar.height)
        }

        radius: 64
        opacity: 1
    }

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
                property bool selected: false;
                id: viewSelection1
                height: width / 1.6
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.rightMargin: 20
                anchors.leftMargin: 20
                anchors.top: parent.top
                anchors.topMargin: 20
                border.color: selected ? "#2196F3" : "white"
                border.width: 4
                GStreamerSurface {
                    id: sideVideoSurface1
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                    backgroundColor: "red"
                }
                DropShadow {
                    source: viewSelection1Text
                    anchors.fill: viewSelection1Text
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
                property bool selected: true;
                id: viewSelection2
                height: width / 1.6
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.rightMargin: 20
                anchors.leftMargin: 20
                anchors.top: viewSelection1.bottom
                anchors.topMargin: 20
                border.color: selected ? "#2196F3" : "white"
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
                property bool selected: false;
                id: viewSelection3
                height: width / 1.6
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.rightMargin: 20
                anchors.leftMargin: 20
                anchors.top: viewSelection2.bottom
                anchors.topMargin: 20
                border.color: selected ? "#2196F3" : "white"
                border.width: 4
                ShaderEffectSource {
                    sourceItem: mapWebEngine
                    sourceRect: Qt.rect(0, 0, mapWebEngine.width, mapWebEngine.height)
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                }
                /*WebEngineView {
                    id: mapWebEngine2
                    anchors.fill: parent
                    url: "qrc:/html/map.html"
                    anchors.margins: parent.border.width
                }*/
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

    DropShadow {
        source: connectionStatusImage2
        anchors.fill: connectionStatusImage2
        radius: 20
        samples: 40
        color: "#66000000"
        visible: sidebar.state == "hidden"
    }

    Image {
        id: connectionStatusImage2
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

    ColorOverlay {
        anchors.fill: connectionStatusImage2
        source: connectionStatusImage2
        color: "#4CAF50"
        visible: sidebar.state == "hidden"
    }

    DropShadow {
        source: pingLabel2
        anchors.fill: pingLabel2
        radius: 20
        samples: 40
        color: "#66000000"
        visible: sidebar.state == "hidden"
    }

    Label {
        id: pingLabel2
        font.pixelSize: 64
        anchors.right: parent.right
        anchors.rightMargin: 10
        anchors.leftMargin: 10
        anchors.left: connectionStatusImage2.right
        anchors.verticalCenter: connectionStatusImage2.verticalCenter
        text: pingLabel.text
        visible: sidebar.state == "hidden"
        color: "#4CAF50"
    }

}
