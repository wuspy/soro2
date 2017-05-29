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
    minimumHeight: videoBounceLabel.y + videoBounceLabel.height
    maximumHeight: videoBounceLabel.y + videoBounceLabel.height
    title: "Mission Control Master"

    property alias videoBounceLabelText: videoBounceLabel.text
    property alias audioBounceLabelText: audioBounceLabel.text

    /*
      Connection status properties
      */
    property bool connected: false
    property int latency: 0
    property int dataRateFromRover: 0
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
        font.pixelSize: 36
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
        font.pixelSize: 36
        anchors.top: connectionStatusImage.bottom
        anchors.right: parent.right
        anchors.rightMargin: 10
        anchors.left: parent.horizontalCenter
        text: connected && latency >= 0 ? latency + " ms" : "---"
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
        font.pixelSize: 36
        color: "black"
        text: {
            if (connected) {
                var uints
                var rate = dataRateFromRover
                if (rate >= 1000000) {
                    uints = "MB/s"
                    rate = Math.round(rate / 10000.0) / 10.0
                }
                else if (rate >= 1000) {
                    uints = "KB/s"
                    rate = Math.round(rate / 10.0) / 10.0
                }
                else {
                    uints = "B/s"
                }
                rate + " " + uints
            }
            else {
                "---"
            }
        }
    }

    Label
    {
        id: audioBounceTitleLabel
        anchors.left: dataRateLabel.left
        anchors.right: pingLabel.right
        anchors.top: pingLabel.bottom
        anchors.topMargin: 20
        text: "Audio Bounce Addresses"
        font.bold: true
    }

    Label
    {
        id: audioBounceLabel
        anchors.left: audioBounceTitleLabel.left
        anchors.right: audioBounceTitleLabel.right
        anchors.top: audioBounceTitleLabel.bottom
        text: "<ul><li>None</li></ul>"
        wrapMode: Text.WordWrap
    }

    Label
    {
        id: videoBounceTitleLabel
        anchors.left: dataRateLabel.left
        anchors.right: pingLabel.right
        anchors.top: audioBounceLabel.bottom
        text: "Video Bounce Addresses"
        font.bold: true
    }

    Label
    {
        id: videoBounceLabel
        anchors.left: videoBounceTitleLabel.left
        anchors.right: videoBounceTitleLabel.right
        anchors.top: videoBounceTitleLabel.bottom
        text: "<ul><li>None</li></ul>"
        wrapMode: Text.WordWrap
    }
}
