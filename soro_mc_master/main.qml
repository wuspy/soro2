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

ApplicationWindow {
    visible: true
    width: 400
    minimumWidth: width
    maximumWidth: width
    height: commActivityLabel.y + commActivityLabel.height
    minimumHeight: height
    maximumHeight: height
    title: "Mission Control Master"

    /*
      Connection status properties
      */
    property bool connected: false
    property int latency: 0
    property int bitrateDown: 0
    property int bitrateUp: 0

    /*
      Internal color properties used throughout the UI
      */
    property color theme_yellow: "#FBC02D"
    property color theme_red: "#d32f2f"
    property color theme_green: "#388E3C"


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
        source: connected ? "qrc:/icons/ic_check_circle_white_48px.svg" : "qrc:/icons/ic_error_white_48px.svg"
    }

    ColorOverlay {
        id: connectionStatusImageColorOverlay
        anchors.fill: connectionStatusImage
        source: connectionStatusImage
        color: connected ? theme_green : theme_yellow
    }

    Label {
        id: connectionStatusLabel
        font.pixelSize: 48
        anchors.right: parent.right
        anchors.rightMargin: 10
        anchors.leftMargin: 10
        anchors.left: connectionStatusImage.right
        anchors.verticalCenter: connectionStatusImage.verticalCenter
        text: connected ? "Connected" : "Connecting..."
        color: connected ? theme_green : theme_yellow
    }

    Label {
        id: pingLabel
        font.pixelSize: 48
        anchors.top: connectionStatusImage.bottom
        anchors.right: parent.right
        anchors.rightMargin: 10
        anchors.left: parent.horizontalCenter
        text: connected && latency > 0 ? latency + "ms" : "---"
        horizontalAlignment: Text.AlignHCenter
        color: "black"
    }

    Label {
        id: bitrateLabel
        anchors.top: connectionStatusImage.bottom
        anchors.left: connectionStatusImage.left
        anchors.right: parent.horizontalCenter
        anchors.bottom: pingLabel.bottom
        horizontalAlignment: Text.AlignHCenter
        font.pixelSize: 24
        color: "black"
        text: {
            if (connected) {
                var upUnits, downUnits;
                if (bpsUp > 1000000) {
                    upUnits = "Mb/s"
                    bpsUp = Math.round(bpsUp / 10000) / 100
                }
                else if (bpsUp > 1000) {
                    upUnits = "Kb/s"
                    bpsUp = Math.round(bpsUp / 10) / 100
                }
                else {
                    upUnits = "b/s"
                }
                if (bpsDown > 1000000) {
                    downUnits = "Mb/s"
                    bpsDown = Math.round(bpsDown / 10000) / 100
                }
                else if (bpsDown > 1000) {
                    downUnits = "Kb/s"
                    bpsDown = Math.round(bpsDown / 10) / 100
                }
                else {
                    downUnits = "b/s"
                }
                "▲ <b>" + bpsUp + "</b> " + upUnits +
                        "<br>" +
                        "▼ <b>" + bpsDown + "</b> " + downUnits
            }
            else {
                "---<br>---"
            }
        }
    }

    Label {
        id: connectedMcsHeaderLabel
        color: "black"
        anchors.top: pingLabel.bottom
        anchors.left: connectionStatusImage.left
        anchors.topMargin: 20
        font.pixelSize: 20
        text: "3 Mission Control(s) connected"
    }

    Label {
        id: connectedMcsLabel
        anchors.top: connectedMcsHeaderLabel.bottom
        anchors.left: connectedMcsHeaderLabel.left
    }

    Label {
        id: mediaStreamsHeaderLabel
        color: "black"
        anchors.top: connectedMcsLabel.bottom
        anchors.left: connectionStatusImage.left
        font.pixelSize: 20
        text: "3 Media stream(s) active"
    }

    Label {
        id: mediaStreamsLabel
        anchors.top: mediaStreamsHeaderLabel.bottom
        anchors.left: mediaStreamsHeaderLabel.left
        text: "<ul><li>Main Camera</li></ul>"
        font.pointSize: 12
    }

    Label {
        id: commActivityHeaderLabel
        color: "black"
        anchors.top: mediaStreamsLabel.bottom
        anchors.left: connectionStatusImage.left
        font.pixelSize: 20
        text: "Communication Activity"
    }
    Label {
        id: commActivityLabel
        anchors.top: commActivityHeaderLabel.bottom
        anchors.left: commActivityHeaderLabel.left
        text: "<ul><li>Main Camera</li></ul>"
        font.pointSize: 12
    }
}
