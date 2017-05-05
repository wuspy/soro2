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
    minimumWidth: 400
    maximumWidth: 400
    minimumHeight: connectedNodesLabel.y + connectedNodesLabel.height
    maximumHeight: connectedNodesLabel.y + connectedNodesLabel.height
    title: "Mission Control Master"

    /*
      Connection status properties
      */
    property bool connected: false
    property int latency: 0
    property int dataRateDown: 0
    property int dataRateUp: 0
    property string connectedNodeInfo

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
        text: connected && latency >= 0 ? latency + "ms" : "---"
        horizontalAlignment: Text.AlignHCenter
        color: "black"
    }

    Label {
        id: dataRateLabel
        anchors.top: connectionStatusImage.bottom
        anchors.left: connectionStatusImage.left
        anchors.right: parent.horizontalCenter
        anchors.bottom: pingLabel.bottom
        horizontalAlignment: Text.AlignHCenter
        font.pixelSize: 24
        color: "black"
        text: {
            if (connected) {
                var upUnits, downUnits
                var up = dataRateUp, down = dataRateDown
                if (up > 1000000) {
                    upUnits = "MB/s"
                    up = Math.round(up / 10000) / 100
                }
                else if (up > 1000) {
                    upUnits = "KB/s"
                    up = Math.round(up / 10) / 100
                }
                else {
                    upUnits = "B/s"
                }
                if (down > 1000000) {
                    downUnits = "MB/s"
                    down = Math.round(down / 10000) / 100
                }
                else if (down > 1000) {
                    downUnits = "KB/s"
                    down = Math.round(down / 10) / 100
                }
                else {
                    downUnits = "B/s"
                }
                "▲ <b>" + up + "</b> " + upUnits +
                        "<br>" +
                        "▼ <b>" + down + "</b> " + downUnits
            }
            else {
                "---<br>---"
            }
        }
    }

    Label {
        id: connectedNodesLabelHeader
        color: "black"
        anchors.top: pingLabel.bottom
        anchors.left: connectionStatusImage.left
        anchors.topMargin: 20
        font.pixelSize: 20
        text: "Connected Nodes"
    }

    Label {
        id: connectedNodesLabel
        anchors.top: connectedNodesLabelHeader.bottom
        anchors.left: connectedNodesLabelHeader.left
        text: connectedNodeInfo
    }
}
