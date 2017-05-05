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
import QtQuick.Controls 2.1
import QtGraphicalEffects 1.0

import "Theme.js" as Theme

Item
{
    id: host
    width: 900
    height: 1200

    /* Width of the notifications
      */
    property alias notificationWidth: list.width

    /* In order for the blur behind the notifications to work, you must specify the item
      from which to source the texture to blur. This item should be behind the notificaiton host,
      and should be nested as a sibling of the notificaiton host.
      */
    property Item blurSource

    /* Padding around the text and icon of the notification
      */
    property int padding: 8

    /* Spacing between each notification in the list
      */
    property int spacing: 8

    function notify(type, title, text)
    {
        notificationListModel.append({"item_type": type, "item_title": title, "item_text": text})
    }

    function dismiss()
    {
        notificationListModel.clear()
    }

    ListView
    {
        id: list
        width: 400
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        height: contentHeight

        model: ListModel
        {
            id: notificationListModel
        }

        delegate: Item
        {
            height: message.height + host.spacing
            anchors.left: parent.left
            anchors.right: parent.right
            Item
            {
                id: message
                anchors.left: parent.left
                anchors.right: parent.right
                height: Math.max(notificationLabel.y + notificationLabel.height + host.padding,
                            notificationImage.y + notificationImage.height + host.padding)

                DropShadow
                {
                    id: notificationShadow
                    anchors.fill: notificationPane
                    anchors.horizontalCenter: parent.horizontalCenter
                    source: notificationPane
                    radius: 32
                    samples: radius
                    color: Theme.shadow
                }

                FastBlur
                {
                    id: notificationBlur
                    anchors.fill: notificationPane
                    source: ShaderEffectSource {
                       sourceItem: host.blurSource
                       sourceRect: Qt.rect(host.x + list.x + x, host.y + list.y + y, notificationBlur.width, notificationBlur.height)
                    }

                    radius: Theme.blurRadius
                }

                Rectangle
                {
                    id: notificationPane
                    anchors.fill: parent
                    //state: "visible"
                    color: Theme.background

                    Text
                    {
                        id: notificationTitleLabel
                        text: item_title
                        font.pixelSize: 20
                        font.bold: true
                        anchors.right: parent.right
                        anchors.rightMargin: host.padding
                        anchors.left: notificationImage.right
                        anchors.leftMargin: 12
                        anchors.top: parent.top
                        anchors.topMargin: host.padding
                        color: Theme.foreground
                        wrapMode: Text.ElideRight
                    }

                    Text
                    {
                        id: notificationLabel
                        text: item_text
                        anchors.topMargin: 0
                        anchors.leftMargin: 12
                        anchors.rightMargin: host.padding
                        anchors.right: parent.right
                        anchors.left: notificationImage.right
                        anchors.top: notificationTitleLabel.bottom
                        verticalAlignment: Text.AlignTop
                        wrapMode: Text.WordWrap
                        font.pixelSize: 14
                        color: Theme.foreground
                    }

                    Image
                    {
                        id: notificationImage
                        height: 96
                        width: height
                        sourceSize.height: height
                        sourceSize.width: width
                        anchors.topMargin: host.padding
                        anchors.leftMargin: host.padding
                        anchors.rightMargin: 8
                        fillMode: Image.PreserveAspectFit
                        anchors.left: parent.left
                        anchors.top: parent.top
                        visible: false
                        source: {
                           switch (item_type) {
                           case "info":
                               "qrc:/icons/ic_info_white_48px.svg"
                               break
                           case "warning":
                               "qrc:/icons/ic_warning_white_48px.svg"
                               break
                           case "error":
                               "qrc:/icons/ic_error_white_48px.svg"
                               break
                           }
                        }
                    }

                    ColorOverlay
                    {
                        id: notificationImageColorOverlay
                        anchors.fill: notificationImage
                        source: notificationImage
                        color: {
                            switch (item_type)
                            {
                            case "info":
                                Theme.blue
                                break
                            case "warning":
                                Theme.yellow
                                break
                            case "error":
                                Theme.red
                                break
                            }
                        }
                    }
                }
            }
        }
    }

    ButtonHint
    {
        anchors.top: list.bottom
        anchors.topMargin: 20
        anchors.right: list.right
        button: "a"
        text: "Dismiss all"
        visible: list.count > 0
    }
}
