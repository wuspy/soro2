import QtQuick 2.7
import QtGraphicalEffects 1.0
Item {
   anchors.horizontalCenter: parent.horizontalCenter
   width: 400
   height: 120
   anchors.margins: 10

   DropShadow {
       id: notificationShadow
       anchors.fill: notificationPane
       anchors.horizontalCenter: parent.horizontalCenter
       source: notificationPane
       radius: 15
       visible: notificationPane.visible
       opacity: notificationPane.opacity
       samples: 20
       color: "#000000"
   }

   Rectangle {
       id: notificationPane
       anchors.fill: parent
       state: "visible"
       color: "#88000000"

       Text {
           id: notificationTitleLabel
           text:"Test Message"
           font.pointSize: 12
           font.bold: true
           anchors.right: parent.right
           anchors.rightMargin: 8
           anchors.left: notificationImage.right
           anchors.leftMargin: 12
           anchors.top: parent.top
           anchors.topMargin: 8
           color: "white"
       }

       Text {
           id: notificationLabel
           text: "This is a test notification."
           anchors.topMargin: 0
           anchors.leftMargin: 12
           anchors.bottom: parent.bottom
           anchors.bottomMargin: 8
           anchors.rightMargin: 8
           anchors.right: parent.right
           anchors.left: notificationImage.right
           anchors.top: notificationTitleLabel.bottom
           verticalAlignment: Text.AlignTop
           wrapMode: Text.WordWrap
           font.pointSize: 10
           color: "white"
       }


       Image {
           id: notificationImage
           width: height
           sourceSize.height: height
           sourceSize.width: width
           anchors.margins: 8
           fillMode: Image.PreserveAspectFit
           anchors.left: parent.left
           anchors.bottom: parent.bottom
           anchors.top: parent.top
           source: "qrc:/icons/ic_check_circle_white_48px.svg"
       }

       ColorOverlay {
           id: notificationImageColorOverlay
           anchors.fill: notificationImage
           source: notificationImage
           color: "#4CAF50"
       }

       MouseArea {
           id: notificationMouseArea
           anchors.fill: parent
           onClicked: notificationPane.state = "hidden"
           cursorShape: Qt.PointingHandCursor
           anchors.topMargin: -notificationPane.topPadding
           anchors.bottomMargin: -notificationPane.bottomPadding
           anchors.leftMargin: -notificationPane.leftPadding
           anchors.rightMargin: -notificationPane.rightPadding
       }

       states: [
           State {
               name: "hidden"
               PropertyChanges {
                   target: notificationPane
                   anchors.rightMargin: -width - notificationShadow.radius
                   opacity: 0
               }
           },
           State {
               name: "visible"
               PropertyChanges {
                   target: notificationPane
                   anchors.rightMargin: 0
                   opacity: 0.7
               }
           }
       ]
   }
}


