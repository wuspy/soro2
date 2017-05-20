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
import QtQuick.Controls 2.1
import QtGraphicalEffects 1.0

import "Theme.js" as Theme

Item
{
    property string button: "a"
    property string key: "a"
    property string buttonTheme: "PS"
    property alias text: btnLabel.text

    width: content.width
    height: 32
    opacity: 0.8

    DropShadow
    {
        source: content
        anchors.fill: content
        radius: 10
        samples: radius
        cached: true
        spread: Theme.shadowSpread
        color: Theme.shadow
    }

    Item
    {
        id: content
        anchors.fill: parent
        width: btnImage.width + btnLabel.width + btnLabel.x

        Image
        {
            id: btnImage
            visible: button != ""
            anchors.left: parent.left
            anchors.verticalCenter: parent.verticalCenter
            source: "qrc:/btn_icons/" + buttonTheme + "/" + button + ".png"
            sourceSize: Qt.size(100, 100)
            height: 32
            width: height
        }

        Label
        {
            id: dividerLabel
            visible: button != ""
            anchors.left: btnImage.right
            anchors.leftMargin: 4
            anchors.verticalCenter: parent.verticalCenter
            color: Theme.foreground
            font.bold: true
            text: "/"
            font.pixelSize: 20
        }

        Image
        {
            id: keyImage
            anchors.left: dividerLabel.right
            anchors.leftMargin: 4
            anchors.verticalCenter: parent.verticalCenter
            source: "qrc:/btn_icons/KB/" + key + ".png"
            sourceSize: Qt.size(100, 100)
            height: 32
            width: height
        }

        Label
        {
            id: btnLabel
            anchors.left: keyImage.right
            anchors.leftMargin: 4
            anchors.verticalCenter: parent.verticalCenter
            color: Theme.foreground
            font.bold: true
            font.pixelSize: 20
        }
    }

}
