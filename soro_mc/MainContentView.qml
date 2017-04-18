import QtQuick 2.0
import QtWebEngine 1.4
import Soro 1.0

Item {

    property alias mapView: mapWebEngine
    property alias video0Surface: video0Surface
    property alias video1Surface: video1Surface
    property alias video2Surface: video2Surface
    property alias video3Surface: video3Surface
    property alias video4Surface: video4Surface
    property alias video5Surface: video5Surface
    property alias video6Surface: video6Surface
    property alias video7Surface: video7Surface
    property alias video8Surface: video8Surface
    property alias video9Surface: video9Surface

    property string activeView: "video0"


    /*
      The web view that shows the Google Maps overlay
      */
    WebEngineView {
        id: mapWebEngine
        anchors.fill: parent
        url: "qrc:/html/map.html"
        enabled: activeView === "map"
    }

    /*
      GStreamer video surfaces, one per camera
      */

    GStreamerSurface {
        id: video0Surface
        anchors.fill: parent
        enabled: activeView === "video0"
        z: enabled ? 1 : 0
    }

    GStreamerSurface {
        id: video1Surface
        anchors.fill: parent
        enabled: activeView === "video1"
        z: enabled ? 1 : 0
    }

    GStreamerSurface {
        id: video2Surface
        anchors.fill: parent
        enabled: activeView === "video2"
        z: enabled ? 1 : 0
    }

    GStreamerSurface {
        id: video3Surface
        anchors.fill: parent
        enabled: activeView === "video3"
        z: enabled ? 1 : 0
    }

    GStreamerSurface {
        id: video4Surface
        anchors.fill: parent
        enabled: activeView === "video4"
        z: enabled ? 1 : 0
    }

    GStreamerSurface {
        id: video5Surface
        anchors.fill: parent
        enabled: activeView === "video5"
        z: enabled ? 1 : 0
    }

    GStreamerSurface {
        id: video6Surface
        anchors.fill: parent
        enabled: activeView === "video6"
        z: enabled ? 1 : 0
    }

    GStreamerSurface {
        id: video7Surface
        anchors.fill: parent
        enabled: activeView === "video7"
        z: enabled ? 1 : 0
    }

    GStreamerSurface {
        id: video8Surface
        anchors.fill: parent
        enabled: activeView === "video8"
        z: enabled ? 1 : 0
    }

    GStreamerSurface {
        id: video9Surface
        anchors.fill: parent
        enabled: activeView === "video9"
        z: enabled ? 1 : 0
    }
}
