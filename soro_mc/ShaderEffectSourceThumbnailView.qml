import QtQuick 2.0
import QtGraphicalEffects 1.0
import Soro 1.0

Rectangle {
    property bool selected: false;
    id: shaderEffectSourceThumbnailView

    property Item shaderSource
    property string text

    onShaderSourceChanged: {
        shaderEffectSource.sourceItem = shaderSource
        shaderEffectSource.sourceRect = Qt.rect(0, 0, shaderSource.width, shaderSource.height)
    }

    width: 200
    height: width / 1.6
    border.color: selected ? "#2196F3" : "white"
    border.width: 4

    ShaderEffectSource {
        id: shaderEffectSource
        anchors.fill: parent
        anchors.margins: parent.border.width
    }

    DropShadow {
        source: label
        anchors.fill: label
        radius: 10
        samples: 20
        color: "black"
    }

    Text {
        id: label
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.margins: 12
        font.pixelSize: 24
        font.bold: true
        color: "white"
        text: parent.text
    }
}
