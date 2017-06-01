import QtQuick 2.7
import QtQuick.Controls 2.0
import Soro 1.0

import "Theme.js" as Theme

Rectangle {
    id: mapView
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
        contentWidth: mapContainer.width
        contentHeight: mapContainer.height

        Item {
            id: mapContainer
            width: Math.max(flickable.width, impl.width)
            height: Math.max(flickable.height, impl.height)

            MapViewImpl {
                id: impl
                anchors.centerIn: parent
            }

            MouseArea {
                id: mouseArea
                width: impl.width
                height: impl.height
                anchors.centerIn: parent
                enabled: mapView.enabled
                hoverEnabled: enabled

                onDoubleClicked: {
                    impl.markPoint(mouse.x, mouse.y)
                }
                onMouseXChanged: {
                    impl.mouseChanged(containsMouse, mouse.x, mouse.y)
                }
                onMouseYChanged: {
                    impl.mouseChanged(containsMouse, mouse.x, mouse.y)
                }

                onExited: {
                    impl.mouseChanged(false, 0, 0)
                }
            }
        }
    }
}
