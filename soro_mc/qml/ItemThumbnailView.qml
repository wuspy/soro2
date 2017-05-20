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

import QtQuick 2.0
import QtGraphicalEffects 1.0
import Soro 1.0

import "Theme.js" as Theme

Rectangle {
    property bool selected: false;
    id: shaderEffectSourceThumbnailView

    property Item shaderSource
    property Rectangle shaderSourceRect
    property string text: ""
    property string streamProfile: ""
    property bool streaming: false

    signal clicked()

    onShaderSourceChanged: {
        shaderEffectSource.sourceItem = shaderSource
    }

    onShaderSourceRectChanged: {
        shaderEffectSource.sourceRect = shaderSourceRect
    }

    width: 200
    height: width / 1.77
    border.color: selected ? Theme.blue : Theme.foreground
    border.width: 4

    ShaderEffectSource {
        id: shaderEffectSource
        anchors.fill: parent
        anchors.margins: parent.border.width
    }

    DropShadow {
        source: overlay
        anchors.fill: overlay
        radius: 10
        samples: 20
        spread: Theme.shadowSpread
        color: Theme.shadow
    }

    Item {
        id: overlay
        anchors.fill: parent

        Text {
            id: label
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.margins: 12
            font.pixelSize: 24
            font.bold: true
            color: Theme.foreground
            text: shaderEffectSourceThumbnailView.text
        }

        ColorOverlay {
            id: profileImageColorOverlay
            anchors.fill: profileImage
            source: profileImage
            color: Theme.red
            visible: shaderEffectSourceThumbnailView.streaming
        }

        Image {
            id: profileImage
            source: "qrc:/icons/ic_play_circle_filled_white_48px.svg"
            visible: false
            anchors.bottom: parent.bottom
            anchors.left: parent.left
            anchors.margins: 12
            width: 24
            height: width
            sourceSize: Qt.size(48, 48)
        }

        Text {
            id: profileLabel
            anchors.verticalCenter: profileImage.verticalCenter
            anchors.left: profileImage.right
            anchors.margins: 12
            font.bold: true
            font.pixelSize: 20
            color: Theme.foreground
            text: shaderEffectSourceThumbnailView.streamProfile.toUpperCase()
            visible: shaderEffectSourceThumbnailView.streaming
        }
    }

    MouseArea {
        anchors.fill: parent
        onClicked: parent.clicked()
    }
}
