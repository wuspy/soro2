import QtQuick 2.7
import QtQuick.Controls 2.0
import Soro 1.0

import "Theme.js" as Theme

Rectangle {
    color: "black"
    property alias impl: impl
    property alias image: impl.image

    Label {
        font.pixelSize: 48
        color: Theme.foreground
        text: "Invalid map image"
        anchors.centerIn: parent
    }

    Flickable {
        id: flickable
        anchors.fill: parent
        contentWidth: impl.width
        contentHeight: impl.height

        MapViewImpl {
            id: impl
        }
    }
}
