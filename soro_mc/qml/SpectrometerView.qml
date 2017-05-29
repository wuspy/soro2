import QtQuick 2.7
import QtCharts 2.0

Rectangle {
    color: "#263238"
    property bool spectrometerOn: false
    property var readingsWhite
    property var readings404

    readonly property int samples: 288
    readonly property int spectralRangeStart: 340
    readonly property int spectralRangeEnd: 850
    readonly property int spectralRange: spectralRangeEnd - spectralRangeStart

    onReadings404Changed: {
        lineSeries404.removePoints(0, 288);
        for (var i = 0; i < 288; ++i)
        {
            lineSeries404.append(spectralRangeStart + (spectralRange / samples) * i, readingsWhite[i])
        }
    }

    onReadingsWhiteChanged: {
        lineSeriesWhite.removePoints(0, 288);
        for (var i = 0; i < 288; ++i)
        {
            lineSeriesWhite.append(spectralRangeStart + (spectralRange / samples) * i, readingsWhite[i])
        }
    }

    ChartView {
        backgroundRoundness: 0
        backgroundColor: "#263238"
        anchors.fill: parent
        anchors.topMargin: 48
        id: chart
        theme: ChartView.ChartThemeDark

        LineSeries {
            id: lineSeriesWhite
            name: "White"
        }

        LineSeries {
            id: lineSeries404
            name: "404nm"
        }
    }
}
