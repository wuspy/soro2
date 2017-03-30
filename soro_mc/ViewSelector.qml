import QtQuick 2.0

Item {
    width: 200
    height: 600

    property int cameraCount: 0
    property string camera1Name: "Camera 1"
    property string camera2Name: "Camera 2"
    property string camera3Name: "Camera 3"
    property string camera4Name: "Camera 4"
    property string camera5Name: "Camera 5"

    CameraThumbnailView {
        id: cameraView1
        width: parent.width * 0.8
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 0
        text: camera1Name
        anchors.top: parent.top
        anchors.topMargin: parent.width * 0.1
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView2
        width: parent.width * 0.8
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 1
        text: camera2Name
        anchors.top: cameraView1.bottom
        anchors.topMargin: parent.width * 0.1
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView3
        width: parent.width * 0.8
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 2
        text: camera3Name
        anchors.top: cameraView2.bottom
        anchors.topMargin: parent.width * 0.1
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView4
        width: parent.width * 0.8
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 3
        text: camera4Name
        anchors.top: cameraView3.bottom
        anchors.topMargin: parent.width * 0.1
        anchors.horizontalCenter: parent.horizontalCenter
    }

    CameraThumbnailView {
        id: cameraView5
        width: parent.width * 0.8
        height: visible ? width / 1.6 : 0
        visible: parent.cameraCount > 4
        text: camera5Name
        anchors.top: cameraView4.bottom
        anchors.topMargin: parent.width * 0.1
        anchors.horizontalCenter: parent.horizontalCenter
    }

    ShaderEffectSourceThumbnailView {
        id: shaderEffectSourceView
        width: parent.width * 0.8
        height: visible ? width / 1.6 : 0
        anchors.top: cameraView5.bottom
        anchors.topMargin: parent.width * 0.1
        anchors.horizontalCenter: parent.horizontalCenter
    }
}
