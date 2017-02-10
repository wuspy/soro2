import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0
import QtGraphicalEffects 1.0

ApplicationWindow  {
    title: "Soro Mission Control"
    width: 1600
    height: 1000
    visible: true

    Image {
        id: mainImage
        anchors.fill: parent
        source: "qrc:/4.png"
    }

    DropShadow {
        anchors.fill: header
        source: header

        radius: 16
        samples: 32
    }

    FastBlur {
        anchors.fill: header
        source: ShaderEffectSource {
            sourceItem: mainImage
            sourceRect: Qt.rect(header.x, header.y, header.width, header.height)
        }

        radius: 64
        opacity: 1
    }

    Rectangle {
        id: header
        anchors.left: parent.left
        anchors.right: parent.right
        height: 200
        opacity: 1
        color: "#88000000"

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
            source: "qrc:/ic_check_circle_white_48px.svg"
        }

        ColorOverlay {
            anchors.fill: connectionStatusImage
            source: connectionStatusImage
            color: "#4CAF50"
        }

        Label {
            id: connectionStatusLabel
            font.pixelSize: 48
            width: 300
            anchors.left: connectionStatusImage.right
            anchors.verticalCenter: connectionStatusImage.verticalCenter
            text: "Connected"
            color: "#4CAF50"
        }

        Label {
            id: pingLabel
            font.pixelSize: 48
            anchors.left: connectionStatusLabel.right
            anchors.verticalCenter: connectionStatusImage.verticalCenter
            text: "18ms"
            width: 200
            color: "white"
        }

        Label {
            id: bitrateLabel
            text: qsTr("▲ <b>3.4</b> Mb/s ▼ <b>9.6</b> Mb/s")
            anchors.top: connectionStatusLabel.bottom
            anchors.left: connectionStatusLabel.left
            font.pixelSize: 24
            color: "white"
        }

        Rectangle {
            id: viewSelectionRectangle
            anchors.fill: parent
            anchors.leftMargin: 600
            color: "transparent"

            Rectangle {
                id: viewSelection1
                width: 256
                height: 160
                anchors.verticalCenter: parent.verticalCenter
                anchors.left: parent.left
                anchors.leftMargin: 20
                border.color: "white"
                border.width: 4
                Image {
                    id: viewSelection1Image
                    source: "qrc:/1.png"
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                }
                DropShadow {
                    source: viewSelection1Text
                    anchors.fill: viewSelection1Text
                    radius: 10
                    samples: 20
                    color: "black"
                }

                Text {
                    id: viewSelection1Text
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 12
                    font.pixelSize: 24
                    font.bold: true
                    color: "white"
                    text: "ArmCam"
                }
            }

            Rectangle {
                id: viewSelection2
                width: 256
                height: 160
                anchors.left: viewSelection1.right
                anchors.leftMargin: 20
                anchors.verticalCenter: parent.verticalCenter
                border.color: "#2196F3"
                border.width: 4
                Image {
                    id: viewSelection2Image
                    source: "qrc:/4.png"
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                }
                DropShadow {
                    source: viewSelection2Text
                    anchors.fill: viewSelection2Text
                    radius: 10
                    samples: 20
                    color: "black"
                }

                Text {
                    id: viewSelection2Text
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 12
                    font.pixelSize: 24
                    font.bold: true
                    color: "white"
                    text: "DriveCam"
                }
            }

            Rectangle {
                id: viewSelection3
                width: 256
                height: 160
                anchors.left: viewSelection2.right
                anchors.leftMargin: 20
                anchors.verticalCenter: parent.verticalCenter
                border.color: "white"
                border.width: 4
                Image {
                    id: viewSelection3Image
                    source: "qrc:/5.png"
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                }
                DropShadow {
                    source: viewSelection3Text
                    anchors.fill: viewSelection3Text
                    radius: 10
                    samples: 20
                    color: "black"
                }

                Text {
                    id: viewSelection3Text
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 12
                    font.pixelSize: 24
                    font.bold: true
                    color: "white"
                    text: "GimbalCam"
                }
            }

            Rectangle {
                id: viewSelection4
                width: 256
                height: 160
                anchors.left: viewSelection3.right
                anchors.leftMargin: 20
                anchors.verticalCenter: parent.verticalCenter
                border.color: "white"
                border.width: 4
                Image {
                    id: viewSelection4Image
                    source: "qrc:/8.png"
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                }
                DropShadow {
                    source: viewSelection4Text
                    anchors.fill: viewSelection4Text
                    radius: 10
                    samples: 20
                    color: "black"
                }

                Text {
                    id: viewSelection4Text
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 12
                    font.pixelSize: 24
                    font.bold: true
                    color: "white"
                    text: "GimbalCam2"
                }
            }

            Rectangle {
                id: viewSelection5
                width: 256
                height: 160
                anchors.left: viewSelection4.right
                anchors.leftMargin: 20
                anchors.verticalCenter: parent.verticalCenter
                border.color: "white"
                border.width: 4
                Image {
                    id: viewSelection5Image
                    source: "qrc:/6.png"
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                }
                DropShadow {
                    source: viewSelection5Text
                    anchors.fill: viewSelection5Text
                    radius: 10
                    samples: 20
                    color: "black"
                }

                Text {
                    id: viewSelection5Text
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 12
                    font.pixelSize: 24
                    font.bold: true
                    color: "white"
                    text: "Map"
                }
            }
        }

        Rectangle {
            id: separator1
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.right: viewSelectionRectangle.left
            width: 2
            color: "white"
            opacity: 0.2
        }
    }

}
