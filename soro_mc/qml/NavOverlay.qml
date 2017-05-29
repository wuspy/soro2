import QtQuick 2.0
import QtQuick.Controls 2.0
import QtGraphicalEffects 1.0
import QtQuick.Controls.Material 2.0

import "Theme.js" as Theme

Item {
    width: gpsLabel.x + gpsLabel.width

    property real compassHeading: 0
    property real latitude: 0
    property real longitude: 0

    function degToDms (deg) {
       var d = Math.floor (deg);
       var minfloat = (deg-d)*60;
       var m = Math.floor(minfloat);
       var secfloat = (minfloat-m)*60;
       var s = Math.round(secfloat * 100) / 100;

       if (s==60) {
         m++;
         s=0;
       }
       if (m==60) {
         d++;
         m=0;
       }
       return ("" + d + "° " + m + "' " + s + "\"");
    }

    ColorOverlay {
        anchors.fill: navDirectionImage
        source: navDirectionImage
        color: Theme.foreground
    }

    Image {
        id: navDirectionImage
        anchors.top: parent.top
        anchors.left: parent.left
        source: "qrc:/icons/ic_navigation_white_48px.svg"
        rotation: compassHeading
        sourceSize: Qt.size(width, height)
        height: parent.height
        width: height
    }

    Label
    {
        id: gpsTitleLabel
        anchors.left: navDirectionImage.right
        anchors.leftMargin: 8
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        font.pixelSize: height / 2.8
        color: Theme.foreground
        text: "Lat:<br>Lng:"
    }

    Label
    {
        id: gpsLabel
        width: 150
        horizontalAlignment: Text.AlignRight
        anchors.left: gpsTitleLabel.right
        anchors.leftMargin: 8
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        font.pixelSize: height / 2.8
        color: Theme.foreground
        text: degToDms(latitude) + "<br>" + degToDms(longitude)
    }
}
