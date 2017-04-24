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


import QtQuick 2.0

Item {
    width: 200
    height: 600

    property string selectedView: "video0"

    property int videoCount: 0
    property alias video0Name: video0ThumbnailView.text
    property alias video1Name: video1ThumbnailView.text
    property alias video2Name: video2ThumbnailView.text
    property alias video3Name: video3ThumbnailView.text
    property alias video4Name: video4ThumbnailView.text
    property alias video5Name: video5ThumbnailView.text
    property alias video6Name: video6ThumbnailView.text
    property alias video7Name: video7ThumbnailView.text
    property alias video8Name: video8ThumbnailView.text
    property alias video9Name: video9ThumbnailView.text

    property alias mapThumbnailViewSource: mapThumbnailView.shaderSource
    property alias video0ThumbnailViewSource: video0ThumbnailView.shaderSource
    property alias video1ThumbnailViewSource: video1ThumbnailView.shaderSource
    property alias video2ThumbnailViewSource: video2ThumbnailView.shaderSource
    property alias video3ThumbnailViewSource: video3ThumbnailView.shaderSource
    property alias video4ThumbnailViewSource: video4ThumbnailView.shaderSource
    property alias video5ThumbnailViewSource: video5ThumbnailView.shaderSource
    property alias video6ThumbnailViewSource: video6ThumbnailView.shaderSource
    property alias video7ThumbnailViewSource: video7ThumbnailView.shaderSource
    property alias video8ThumbnailViewSource: video8ThumbnailView.shaderSource
    property alias video9ThumbnailViewSource: video9ThumbnailView.shaderSource

    property int itemMargins: 10

    readonly property string selectedViewTitle: {
        switch (selectedView) {
        case "video0": return video0Name
        case "video1": return video1Name
        case "video2": return video2Name
        case "video3": return video3Name
        case "video4": return video4Name
        case "video5": return video5Name
        case "video6": return video6Name
        case "video7": return video7Name
        case "video8": return video8Name
        case "video9": return video9Name
        case "map": return "Map"
        }
    }

    Flickable {
        id: flickable
        clip: true
        anchors.fill: parent

        contentHeight: mapThumbnailView.y + mapThumbnailView.height + itemMargins

        ItemThumbnailView {
            id: video0ThumbnailView
            width: parent.width
            visible: videoCount > 0
            selected: selectedView === "video0"
            height: visible ? width / 1.6 : 0
            anchors.top: parent.top
            anchors.topMargin: visible ? itemMargins : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onSelectedChanged: {
                if (selected) {
                    flickable.contentY = y - anchors.topMargin
                }
            }
        }

        ItemThumbnailView {
            id: video1ThumbnailView
            width: parent.width
            visible: videoCount > 1
            selected: selectedView === "video1"
            height: visible ? width / 1.6 : 0
            anchors.top: video0ThumbnailView.bottom
            anchors.topMargin: visible ? itemMargins : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onSelectedChanged: {
                if (selected) {
                    flickable.contentY = y - anchors.topMargin
                }
            }
        }

        ItemThumbnailView {
            id: video2ThumbnailView
            width: parent.width
            visible: videoCount > 2
            selected: selectedView === "video2"
            height: visible ? width / 1.6 : 0
            anchors.top: video1ThumbnailView.bottom
            anchors.topMargin: visible ? itemMargins : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onSelectedChanged: {
                if (selected) {
                    flickable.contentY = y - anchors.topMargin
                }
            }
        }

        ItemThumbnailView {
            id: video3ThumbnailView
            width: parent.width
            visible: videoCount > 3
            selected: selectedView === "video3"
            height: visible ? width / 1.6 : 0
            anchors.top: video2ThumbnailView.bottom
            anchors.topMargin: visible ? itemMargins : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onSelectedChanged: {
                if (selected) {
                    flickable.contentY = y - anchors.topMargin
                }
            }
        }

        ItemThumbnailView {
            id: video4ThumbnailView
            width: parent.width
            visible: videoCount > 4
            selected: selectedView === "video4"
            height: visible ? width / 1.6 : 0
            anchors.top: video3ThumbnailView.bottom
            anchors.topMargin: visible ? itemMargins : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onSelectedChanged: {
                if (selected) {
                    flickable.contentY = y - anchors.topMargin
                }
            }
        }

        ItemThumbnailView {
            id: video5ThumbnailView
            width: parent.width
            visible: videoCount > 5
            selected: selectedView === "video5"
            height: visible ? width / 1.6 : 0
            anchors.top: video4ThumbnailView.bottom
            anchors.topMargin: visible ? itemMargins : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onSelectedChanged: {
                if (selected) {
                    flickable.contentY = y - anchors.topMargin
                }
            }
        }

        ItemThumbnailView {
            id: video6ThumbnailView
            width: parent.width
            visible: videoCount > 6
            selected: selectedView === "video6"
            height: visible ? width / 1.6 : 0
            anchors.top: video5ThumbnailView.bottom
            anchors.topMargin: visible ? itemMargins : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onSelectedChanged: {
                if (selected) {
                    flickable.contentY = y - anchors.topMargin
                }
            }
        }

        ItemThumbnailView {
            id: video7ThumbnailView
            width: parent.width
            visible: videoCount > 7
            selected: selectedView === "video7"
            height: visible ? width / 1.6 : 0
            anchors.top: video6ThumbnailView.bottom
            anchors.topMargin: visible ? itemMargins : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onSelectedChanged: {
                if (selected) {
                    flickable.contentY = y - anchors.topMargin
                }
            }
        }

        ItemThumbnailView {
            id: video8ThumbnailView
            width: parent.width
            visible: videoCount > 8
            selected: selectedView === "video8"
            height: visible ? width / 1.6 : 0
            anchors.top: video7ThumbnailView.bottom
            anchors.topMargin: visible ? itemMargins : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onSelectedChanged: {
                if (selected) {
                    flickable.contentY = y - anchors.topMargin
                }
            }
        }

        ItemThumbnailView {
            id: video9ThumbnailView
            width: parent.width
            visible: videoCount > 9
            selected: selectedView === "video9"
            height: visible ? width / 1.6 : 0
            anchors.top: video8ThumbnailView.bottom
            anchors.topMargin: visible ? itemMargins : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onSelectedChanged: {
                if (selected) {
                    flickable.contentY = y - anchors.topMargin
                }
            }
        }

        ItemThumbnailView {
            id: mapThumbnailView
            width: parent.width
            selected: selectedView === "map"
            height: visible ? width / 1.6 : 0
            anchors.top: video9ThumbnailView.bottom
            anchors.topMargin: visible ? itemMargins : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onSelectedChanged: {
                if (selected) {
                    flickable.contentY = y - anchors.topMargin
                }
            }
            text: "Map"
        }
    }
}
