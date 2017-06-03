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
    property int satellites: 0

    function degToDegreeMinutes (deg) {
       var d = d > 0 ? Math.floor (deg) : Math.ceil(deg);
       var m = Math.abs(deg - d) * 60;
       m = Math.round(m * 1000) / 1000;
       return ("" + d + "° " + m + "'");
    }

    ColorOverlay {
        anchors.fill: navDirectionImage
        source: navDirectionImage
        color: Theme.foreground
        rotation: compassHeading
    }

    Image {
        id: navDirectionImage
        anchors.top: parent.top
        anchors.left: parent.left
        source: "qrc:/icons/ic_navigation_white_48px.svg"
        visible: false
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
        font.pixelSize: parent.height / 4.2
        color: Theme.foreground
        text: "Lat:<br>Lng:"
    }

    Label
    {
        id: gpsLabel
        width: 100
        horizontalAlignment: Text.AlignRight
        anchors.left: gpsTitleLabel.right
        anchors.leftMargin: 8
        anchors.top: parent.top
        font.pixelSize: parent.height / 4.2
        color: Theme.foreground
        text: degToDegreeMinutes(latitude) + "<br>" + degToDegreeMinutes(longitude)
    }

    Label
    {
        id: satellitesTitleLabel
        anchors.left: navDirectionImage.right
        anchors.leftMargin: 8
        anchors.top: gpsTitleLabel.bottom
        font.pixelSize: parent.height / 4.2
        color: Theme.foreground
        text: "Satellites:"
    }

    Label
    {
        id: satellitesLabel
        horizontalAlignment: Text.AlignRight
        anchors.left: satellitesTitleLabel.right
        anchors.leftMargin: 8
        anchors.top: gpsTitleLabel.bottom
        anchors.right: parent.right
        font.pixelSize: parent.height / 4.2
        color: Theme.foreground
        text: satellites
    }

}
