import QtQuick 2.7
import Soro 1.0

Flickable {
    id: flickable
    property alias impl: impl

    MapViewImpl {
        id: impl
    }
}
