import QtQuick 2.0
import QtQuick.Controls 2.1
import QtGraphicalEffects 1.0

import "Theme.js" as Theme

Item
{
    property string button: "a"
    property string buttonTheme: "PS"
    property alias text: btnLabel.text
    width: content.width
    opacity: 0.8

    Item
    {
        id: content
        anchors.fill: parent
        width: btnImage.width + btnLabel.width + btnLabel.x

        DropShadow
        {
            source: btnImage
            anchors.fill: btnImage
            radius: 12
            samples: radius
            color: Theme.shadow
        }

        Image
        {
            id: btnImage
            anchors.left: parent.left
            anchors.verticalCenter: parent.verticalCenter
            source: "qrc:/btn_icons/" + buttonTheme + "/" + button + ".png"
            sourceSize: Qt.size(100, 100)
            height: 36
            width: height
        }

        DropShadow
        {
            source: btnLabel
            anchors.fill: btnLabel
            radius: 12
            samples: radius
            color: Theme.shadow
        }

        Label
        {
            id: btnLabel
            anchors.left: btnImage.right
            anchors.leftMargin: 4
            anchors.verticalCenter: parent.verticalCenter
            color: Theme.foreground
            font.bold: true
            font.pixelSize: 24
        }
    }

}
