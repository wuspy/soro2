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
import QtQuick.Controls 2.1
import QtGraphicalEffects 1.0

import "Theme.js" as Theme

Item {
    id: host
    width: 400
    height: 200
    property alias dismissButtonHint: buttonHint.button
    property alias dismissKeyHint: buttonHint.key
    property alias dismissButtonHintTheme: buttonHint.buttonTheme

    /* In order for the blur behind the notifications to work, you must specify the item
      from which to source the texture to blur. This item should be behind the notificaiton host,
      and should be nested as a sibling of the notificaiton host.
      */
    property Item blurSource

    /* Width of the notifications
      */
    property int notificationWidth: 400

    /* Padding around the text and icon of the notification
      */
    property int padding: 8

    property var queue: [];

    function notify(level, title, message) {
        if (messageItem.state == "visible") {
            var remaining = queue.push({"level": level, "title": title, "message": message })
        }
        else {
            messageItem.level = level
            messageItem.title = title
            messageItem.message = message
            messageItem.state = "visible"
        }

        if (queue.length > 0) {
            remainingLabel.text = "+" + queue.length + " more"
        }
        else {
            remainingLabel.text = ""
        }
    }

    function dismiss() {
        messageItem.state = "hidden"
        if (queue.length > 0) {
            var notification = queue.shift()
            notify(notification.level, notification.title, notification.message)
        }
    }

    Item {
        id: messageItem
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        height: buttonHint.y + buttonHint.height
        state: "hidden"
        width: notificationWidth

        property string level: "info"
        property alias title: notificationTitleLabel.text
        property alias message: notificationLabel.text

        states: [
            State {
                name: "visible"
                PropertyChanges {
                    target: messageItem
                    opacity: 1
                }
            },
            State {
                name: "hidden"
                PropertyChanges {
                    target: messageItem
                    opacity: 0
                }
            }
        ]

        transitions: [
            Transition {
                from: "visible"
                to: "hidden"
                PropertyAnimation {
                    properties: "opacity"
                    duration: 100
                    easing.type: Easing.OutQuad
                }
            },
            Transition {
                from: "hidden"
                to: "visible"
                PropertyAnimation {
                    properties: "opacity"
                    duration: 100
                    easing.type: Easing.InQuad
                }
            }
        ]

        DropShadow {
            id: notificationShadow
            anchors.fill: notificationPane
            source: notificationPane
            radius: 32
            samples: radius
            color: Theme.shadow
        }

        FastBlur {
            id: notificationBlur
            anchors.fill: notificationPane
            source: ShaderEffectSource {
               sourceItem: host.blurSource
               sourceRect: Qt.rect(host.x + messageItem.x + x, host.y + messageItem.y + y, notificationBlur.width, notificationBlur.height)
            }

            radius: Theme.blurRadius
        }

        Rectangle {
            id: notificationPane
            width: host.notificationWidth
            height: Math.max(notificationLabel.y + notificationLabel.height + host.padding,
                             notificationImage.y + notificationImage.height + host.padding)
            color: Theme.background

            Text {
                id: notificationTitleLabel
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

            Text {
                id: notificationLabel
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

            Image {
                id: notificationImage
                height: 64
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
                   switch (messageItem.level) {
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

            ColorOverlay {
                id: notificationImageColorOverlay
                anchors.fill: notificationImage
                source: notificationImage
                color: {
                    switch (messageItem.level) {
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

        Label {
            id: remainingLabel
            anchors.top: notificationPane.bottom
            anchors.right: notificationPane.right
            anchors.topMargin: 4
            color: Theme.foreground
            font.bold: true
            font.pixelSize: 20
        }

        ButtonHint {
            id: buttonHint
            anchors.top: notificationPane.bottom
            anchors.topMargin: 4
            anchors.left: notificationPane.left
            text: "Dismiss"
            button: "dp_right"
            key: "enter"
        }
    }
}
