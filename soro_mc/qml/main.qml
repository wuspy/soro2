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
import QtQuick.Controls 2.0
import QtQuick.Controls.Material 2.0
import QtQuick.Layouts 1.0
import QtGraphicalEffects 1.0
import QtWebEngine 1.4
import Soro 1.0

import "Theme.js" as Theme

ApplicationWindow {
    id: root
    visible: true
    width: 800
    height: 600
    title: "Mission Control - " + configuration

    /*
      State of the sidebar, can be either 'hidden' or 'visible'
      */
    property alias sidebarState: sidebar.state

    /*
      Connection status properties
      */
    property bool connected: false
    property int latency: 0
    property int dataRateFromRover: 0
    property string configuration: "Observer"

    /*
      Navigation properties
      */
    property alias compassHeading: navOverlay.compassHeading
    property alias latitude: navOverlay.latitude
    property alias longitude: navOverlay.longitude

    /*
      Selected view in the UI, can be eselectedViewither 'map' or 'camera0'-'camera9'
      */
    property int selectedViewIndex: 0

    /*
      Number of videos available to show
      */
    property int videoCount: 0

    /*
      Total number of views, number of videos +1 for the map
      */
    readonly property int viewCount: mainContentView.viewCount

    function setVideoName(index, name) {
        sidebarViewSelector.setViewTitle(index, name)
    }

    function setVideoIsStreaming(index, streaming) {
        sidebarViewSelector.setViewIsStreaming(index, streaming)
    }

    function setVideoProfileName(index, name) {
        sidebarViewSelector.setViewStreamProfileName(index, name)
    }

    function getVideoSurface(index) {
        return mainContentView.videoSurfaces[index]
    }

    /*
      Audio properties
      */
    property string audioProfile: ""
    property bool audioStreaming: false
    property bool audioMuted: false

    property alias mainContentView: mainContentView
    property alias mapImage: mainContentView.mapImage
    property alias mapViewImpl: mainContentView.mapViewImpl

    /* Emitted when a key is pressed in the UI
      */
    signal keyPressed(int key)

    /* Fullscreen state of the application, boolean
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

    function dismissNotification() {
        notificationHost.dismiss()
    }

    function toggleSidebar() {
        switch (sidebarState) {
        case "visible":
            sidebarState = "hidden"
            break
        case "hidden":
        default:
            sidebarState = "visible"
            break
        }
    }

    function selectViewBelow() {
        if (selectedViewIndex == viewCount - 1) {
            selectedViewIndex = 0
        }
        else {
            selectedViewIndex++
        }
    }

    function selectViewAbove() {
        if (selectedViewIndex == 0) {
            selectedViewIndex = viewCount - 1
        }
        else {
            selectedViewIndex--
        }
    }

    onVideoCountChanged: {
        mainContentView.videoCount = videoCount
        sidebarViewSelector.clearItems()
        for (var i = 0; i < videoCount; ++i) {
            sidebarViewSelector.addItem(mainContentView.videoSurfaces[i], "Video " + i)
        }
        sidebarViewSelector.addItem(mainContentView.mapView, "Map")
        sidebarViewSelector.addItem(mainContentView.spectrometerView, "Spectrometer")
        selectedViewIndex = 0
    }

    onSelectedViewIndexChanged: {
        sidebarViewSelector.selectedViewIndex = selectedViewIndex
        mainContentView.activeViewIndex = selectedViewIndex
    }

    function notify(type, title, text) {
        notificationHost.notify(type, title, text)
    }

    MainContentView {
        id: mainContentView
        anchors.fill: parent
        //activeViewIndex: sidebarViewSelector.selectedViewIndex
    }

    DropShadow {
        source: hudOverlay
        anchors.fill: hudOverlay
        radius: 10
        samples: 20
        spread: Theme.shadowSpread
        color: Theme.shadow
    }

    Item {
        id: hudOverlay
        anchors.fill: parent

        NavOverlay {
            id: navOverlay
            anchors.topMargin: 10
            anchors.rightMargin: 10
            anchors.top: parent.top
            anchors.right: parent.right
            height: 64
        }

        /*PitchRollView {
            width: 300
            height: width
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 10
        }*/

        /*
          Color overlay to colorize the mini connection status image
          */
        ColorOverlay {
            id: miniConnectionStatusImageColorOverlay
            anchors.fill: miniConnectionStatusImage
            source: miniConnectionStatusImage
            color: connectionStatusImageColorOverlay.color
            visible: sidebar.state == "hidden"
        }

        /*
          Status image that is shown when the main sidebar is hidden
          */
        Image {
            id: miniConnectionStatusImage
            width: 64
            height: 64
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.leftMargin: 10
            anchors.topMargin: 10
            visible: false
            sourceSize: Qt.size(width, height)
            source: connectionStatusImage.source
        }

        Text {
            id: miniPingLabel
            visible: sidebar.state == "hidden"
            color: Theme.foreground
            font.pixelSize: 48
            text: pingLabel.text
            anchors.leftMargin: 10
            anchors.verticalCenter: miniConnectionStatusImage.verticalCenter
            anchors.left: miniConnectionStatusImage.right
        }

        Text {
            id: activeViewLabel
            color: Theme.foreground
            font.pixelSize: 48
            text: sidebarViewSelector.selectedViewTitle
            anchors.horizontalCenter: parent.horizontalCenter
        }

        Item {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: activeViewLabel.bottom
            anchors.topMargin: 4
            height: 32
            width: profileLabel.x + profileLabel.width

            ColorOverlay {
                id: profileImageColorOverlay
                anchors.fill: profileImage
                source: profileImage
                color: Theme.red
                visible: sidebarViewSelector.selectedViewIsStreaming
            }

            Image {
                id: profileImage
                source: "qrc:/icons/ic_play_circle_filled_white_48px.svg"
                visible: false
                anchors.top: parent.top
                anchors.left: parent.left
                width: parent.height
                height: parent.height
                sourceSize: Qt.size(48, 48)
            }

            Text {
                id: profileLabel
                anchors.verticalCenter: profileImage.verticalCenter
                anchors.left: profileImage.right
                anchors.leftMargin: 4
                font.bold: true
                font.pixelSize: 24
                color: Theme.foreground
                text: sidebarViewSelector.selectedViewStreamProfile.toUpperCase()
                visible: sidebarViewSelector.selectedViewIsStreaming
            }
        }
    }

    /*
      Drop shadow for the sidebar
      */
    DropShadow {
        anchors.fill: sidebar
        source: sidebar
        spread: Theme.shadowSpread
        color: Theme.shadow
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
            sourceItem: mainContentView
            sourceRect: Qt.rect(sidebar.x, sidebar.y, sidebar.width, sidebar.height)
        }

        radius: Theme.blurRadius
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
        color: Theme.background

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
                    easing.type: Easing.OutExpo
                }
                PropertyAnimation {
                    properties: "opacity"
                    duration: 400
                    easing.type: Easing.OutExpo
                }
            },
            Transition {
                from: "hidden"
                to: "visible"
                PropertyAnimation {
                    properties: "anchors.leftMargin"
                    duration: 200
                    easing.type: Easing.OutExpo
                }
                PropertyAnimation {
                    properties: "opacity"
                    duration: 300
                    easing.type: Easing.OutExpo
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
            source: connected ? "qrc:/icons/ic_check_circle_white_48px.svg" : "qrc:/icons/ic_error_white_48px.svg"
        }

        ColorOverlay {
            id: connectionStatusImageColorOverlay
            anchors.fill: connectionStatusImage
            source: connectionStatusImage
            color: connected ? Theme.green : Theme.yellow
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
            color: connected ? Theme.green : Theme.yellow
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
            color: Theme.foreground
        }

        Label {
            id: dataRateLabel
            anchors.top: connectionStatusImage.bottom
            anchors.left: connectionStatusImage.left
            anchors.right: parent.horizontalCenter
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: 36
            color: Theme.foreground
            text: {
                if (connected) {
                    var uints
                    var rate = dataRateFromRover
                    if (rate >= 1000000) {
                        uints = "MB/s"
                        rate = Math.round(rate / 100000.0) / 10.0
                    }
                    else if (rate >= 1000) {
                        uints = "KB/s"
                        rate = Math.round(rate / 100.0) / 10.0
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

        Image {
            id: audioImage
            width: 32
            height: width
            anchors.top: dataRateLabel.bottom
            anchors.horizontalCenter: connectionStatusImage.horizontalCenter
            anchors.topMargin: 4
            source: audioStreaming ? "qrc:/icons/ic_volume_up_white_48px.svg" : "qrc:/icons/ic_volume_off_white_48px.svg"
            sourceSize: Qt.size(48, 48)
        }

        Label {
            id: audioLabel
            anchors.verticalCenter: audioImage.verticalCenter
            anchors.left: connectionStatusLabel.left
            text: "Audio " + (audioStreaming ? (audioProfile + (audioMuted ? " [MUTE]" : "")) : " off")
            font.bold: true
            font.pixelSize: 20
            color: Theme.foreground
        }

        ViewSelector {
            id: sidebarViewSelector
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: audioImage.bottom
            anchors.bottom: parent.bottom
            spacing: parent.width / 20
            anchors.leftMargin: spacing
            anchors.rightMargin: spacing
            aspectRatio: root.width / root.height

            onViewClicked: {
                root.selectedViewIndex = index
            }
        }
    }

    ButtonHint {
        id: sidebarToggleButtonHint
        visible: sidebar.state == "visible"
        anchors.left: sidebar.right
        anchors.bottom: parent.bottom
        anchors.margins: 8
        button: "dp_left"
        key: "left"
        text: "Toggle sidebar"
    }

    NotificationHost {
        id: notificationHost
        notificationWidth: 400
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        blurSource: mainContentView
    }

    Item {
        id: keyItem
        focus: true
        onFocusChanged: focus = true
        Keys.onPressed: {
            keyPressed(event.key)
        }
    }
}
