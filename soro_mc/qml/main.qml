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

import "Theme.js" as Theme

ApplicationWindow {
    visible: true
    width: 800
    height: 600
    title: "Mission Control"

    /*
      State of the sidebar, can be either 'hidden' or 'visible'
      */
    property alias sidebarState: sidebar.state

    /*
      Connection status properties
      */
    property bool connected: false
    property int latency: 0
    property int dataRateUp: 0
    property int dataRateDown: 0

    /*
      Selected view in the UI, can be either 'map' or 'camera0'-'camera9'
      */
    property alias selectedView: sidebarViewSelector.selectedView

    /*
      Number of videos available to show
      */
    property alias videoCount: sidebarViewSelector.videoCount

    /*
      Names used to label each video with
      */
    property alias video0Name: sidebarViewSelector.video0Name
    property alias video1Name: sidebarViewSelector.video1Name
    property alias video2Name: sidebarViewSelector.video2Name
    property alias video3Name: sidebarViewSelector.video3Name
    property alias video4Name: sidebarViewSelector.video4Name
    property alias video5Name: sidebarViewSelector.video5Name
    property alias video6Name: sidebarViewSelector.video6Name
    property alias video7Name: sidebarViewSelector.video7Name
    property alias video8Name: sidebarViewSelector.video8Name
    property alias video9Name: sidebarViewSelector.video9Name

    /*
      GStreamer surfaces for each video
      */
    property alias video0Surface: mainContentView.video0Surface
    property alias video1Surface: mainContentView.video1Surface
    property alias video2Surface: mainContentView.video2Surface
    property alias video3Surface: mainContentView.video3Surface
    property alias video4Surface: mainContentView.video4Surface
    property alias video5Surface: mainContentView.video5Surface
    property alias video6Surface: mainContentView.video6Surface
    property alias video7Surface: mainContentView.video7Surface
    property alias video8Surface: mainContentView.video8Surface
    property alias video9Surface: mainContentView.video9Surface

    /* Emitted when a key is pressed in the UI
      */
    signal keyPressed(int key)

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

    function dismissNotifications()
    {
        notificationHost.dismiss()
    }

    function toggleSidebar()
    {
        switch (sidebarState)
        {
        case "visible":
            sidebarState = "hidden"
            break
        case "hidden":
        default:
            sidebarState = "visible"
            break
        }
    }

    function selectViewBelow()
    {
        switch (selectedView)
        {
        case "video0":
            selectedView = videoCount > 1 ? "video1" : "map"
            break
        case "video1":
            selectedView = videoCount > 2 ? "video2" : "map"
            break
        case "video2":
            selectedView = videoCount > 3 ? "video3" : "map"
            break
        case "video3":
            selectedView = videoCount > 4 ? "video4" : "map"
            break
        case "video4":
            selectedView = videoCount > 5 ? "video5" : "map"
            break
        case "video5":
            selectedView = videoCount > 6 ? "video6" : "map"
            break
        case "video6":
            selectedView = videoCount > 7 ? "video7" : "map"
            break
        case "video7":
            selectedView = videoCount > 8 ? "video8" : "map"
            break
        case "video8":
            selectedView = videoCount > 9 ? "video9" : "map"
            break
        case "video9":
            selectedView = "map"
            break
        case "map":
            selectedView = videoCount > 0 ? "video0" : "map"
            break
        }
    }

    function selectViewAbove()
    {
        switch (selectedView)
        {
        case "video0":
            selectedView = "map"
            break
        case "video1":
            selectedView = "video0"
            break
        case "video2":
            selectedView = "video1"
            break
        case "video3":
            selectedView = "video2"
            break
        case "video4":
            selectedView = "video3"
            break
        case "video5":
            selectedView = "video4"
            break
        case "video6":
            selectedView = "video5"
            break
        case "video7":
            selectedView = "video6"
            break
        case "video8":
            selectedView = "video7"
            break
        case "video9":
            selectedView = "video8"
            break
        case "map":
            selectedView = videoCount > 0 ? "video" + (videoCount - 1) : "map"
            break
        }
    }

    Item {
        id: keyItem
        focus: true
        Keys.onPressed: {
            keyPressed(event.key)
        }
    }

    function notify(type, title, text) {
        notificationHost.notify(type, title, text)
    }

    MainContentView {
        id: mainContentView
        anchors.fill: parent
        activeView: sidebarViewSelector.selectedView
    }

    /*
      Drop shadow for the mini connection status image
      */
    DropShadow {
        source: miniConnectionStatusImageColorOverlay
        anchors.fill: miniConnectionStatusImageColorOverlay
        radius: 20
        samples: 20
        visible: miniConnectionStatusImageColorOverlay.visible
    }

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

    DropShadow {
        source: miniPingLabel
        anchors.fill: miniPingLabel
        radius: 10
        samples: 20
        color: Theme.shadow
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

    DropShadow {
        source: activeViewLabel
        anchors.fill: activeViewLabel
        radius: 10
        samples: 20
        color: Theme.shadow
    }

    Text {
        id: activeViewLabel
        color: Theme.foreground
        font.pixelSize: 48
        anchors.rightMargin: 10
        anchors.right: parent.right
        anchors.verticalCenter: miniConnectionStatusImage.verticalCenter
        text: sidebarViewSelector.selectedViewTitle
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
            font.pixelSize: 48
            anchors.top: connectionStatusImage.bottom
            anchors.right: parent.right
            anchors.rightMargin: 10
            anchors.left: parent.horizontalCenter
            text: connected && latency >= 0 ? latency + "ms" : "---"
            horizontalAlignment: Text.AlignHCenter
            color: Theme.foreground
        }

        Label {
            id: dataRateLabel
            anchors.top: connectionStatusImage.bottom
            anchors.left: connectionStatusImage.left
            anchors.right: parent.horizontalCenter
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: 24
            color: Theme.foreground
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

        ViewSelector {
            id: sidebarViewSelector
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: dataRateLabel.bottom
            anchors.bottom: parent.bottom
            itemMargins: parent.width / 20
            anchors.leftMargin: itemMargins
            anchors.rightMargin: itemMargins

            mapThumbnailViewSource: mainContentView.mapView
            video0ThumbnailViewSource: mainContentView.video0Surface
            video1ThumbnailViewSource: mainContentView.video1Surface
            video2ThumbnailViewSource: mainContentView.video2Surface
            video3ThumbnailViewSource: mainContentView.video3Surface
            video4ThumbnailViewSource: mainContentView.video4Surface
            video5ThumbnailViewSource: mainContentView.video5Surface
            video6ThumbnailViewSource: mainContentView.video6Surface
            video7ThumbnailViewSource: mainContentView.video7Surface
            video8ThumbnailViewSource: mainContentView.video8Surface
            video9ThumbnailViewSource: mainContentView.video9Surface
        }
    }

    NotificationHost {
        id: notificationHost
        notificationWidth: 500
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.left: sidebar.right
        anchors.right: parent.right
        blurSource: mainContentView
    }
}
