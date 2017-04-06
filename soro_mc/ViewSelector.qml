import QtQuick 2.0

Item {
    width: 200
    height: 600

    property int selectedIndex: 0

    property int cameraCount: 0
    property string camera0Name: "Camera 0"
    property string camera1Name: "Camera 1"
    property string camera2Name: "Camera 2"
    property string camera3Name: "Camera 3"
    property string camera4Name: "Camera 4"
    property string camera5Name: "Camera 5"
    property string camera6Name: "Camera 6"
    property string camera7Name: "Camera 7"
    property string camera8Name: "Camera 8"

    property alias cameraThumbnail0GstreamerSurface: cameraView0.gstreamerSurface
    property alias cameraThumbnail1GstreamerSurface: cameraView1.gstreamerSurface
    property alias cameraThumbnail2GstreamerSurface: cameraView2.gstreamerSurface
    property alias cameraThumbnail3GstreamerSurface: cameraView3.gstreamerSurface
    property alias cameraThumbnail4GstreamerSurface: cameraView4.gstreamerSurface
    property alias cameraThumbnail5GstreamerSurface: cameraView5.gstreamerSurface
    property alias cameraThumbnail6GstreamerSurface: cameraView6.gstreamerSurface
    property alias cameraThumbnail7GstreamerSurface: cameraView7.gstreamerSurface
    property alias cameraThumbnail8GstreamerSurface: cameraView8.gstreamerSurface

    property alias mapThumbnailViewSource: mapThumbnailView.shaderSource

    property int itemMargins: 10

    CameraThumbnailView {
        id: cameraView0
        width: parent.width
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 0
        selected: visible && selectedIndex == 0
        text: camera0Name
        anchors.top: parent.top
        anchors.topMargin: visible ? itemMargins : 0
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView1
        width: parent.width
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 1
        selected: visible && selectedIndex == 1
        text: camera1Name
        anchors.top: cameraView0.bottom
        anchors.topMargin: visible ? itemMargins : 0
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView2
        width: parent.width
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 2
        selected: visible && selectedIndex == 2
        text: camera2Name
        anchors.top: cameraView1.bottom
        anchors.topMargin: visible ? itemMargins : 0
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView3
        width: parent.width
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 3
        selected: visible && selectedIndex == 3
        text: camera3Name
        anchors.top: cameraView2.bottom
        anchors.topMargin: visible ? itemMargins : 0
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView4
        width: parent.width
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 4
        selected: visible && selectedIndex == 4
        text: camera4Name
        anchors.top: cameraView3.bottom
        anchors.topMargin: visible ? itemMargins : 0
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView5
        width: parent.width
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 5
        selected: visible && selectedIndex == 5
        text: camera5Name
        anchors.top: cameraView4.bottom
        anchors.topMargin: visible ? itemMargins : 0
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView6
        width: parent.width
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 6
        selected: visible && selectedIndex == 6
        text: camera6Name
        anchors.top: cameraView5.bottom
        anchors.topMargin: visible ? itemMargins : 0
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView7
        width: parent.width
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 7
        selected: visible && selectedIndex == 7
        text: camera7Name
        anchors.top: cameraView6.bottom
        anchors.topMargin: visible ? itemMargins : 0
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView8
        width: parent.width
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 7
        selected: visible && selectedIndex == 7
        text: camera8Name
        anchors.top: cameraView7.bottom
        anchors.topMargin: visible ? itemMargins : 0
        anchors.horizontalCenter: parent.horizontalCenter
    }

    ShaderEffectSourceThumbnailView {
        id: mapThumbnailView
        width: parent.width
        selected: selectedIndex == cameraCount
        height: visible ? width / 1.6 : 0
        anchors.top: cameraView8.bottom
        anchors.topMargin: visible ? itemMargins : 0
        anchors.horizontalCenter: parent.horizontalCenter
        text: "Map"
    }
}
