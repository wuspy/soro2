#ifndef MAINWINDOWCONTROLLER_H
#define MAINWINDOWCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QQuickWindow>

#include "soro_core/rosnodelist.h"

namespace Soro {

class MainWindowController : public QObject
{
    Q_OBJECT
public:
    explicit MainWindowController(QQmlEngine *engine, const RosNodeList *rosNodeList, QObject *parent = 0);

public Q_SLOTS:
    void onBitrateUpdated(quint64 bitrateUp, quint64 bitrateDown);
    void onLatencyUpdated(quint32 latency);
    void onConnectedChanged(bool connected);

private Q_SLOTS:
    void onRosNodeListUpdated();

private:
    QQuickWindow *_window = nullptr;
    const RosNodeList *_rosNodeList;
};

} // namespace Soro

#endif // MAINWINDOWCONTROLLER_H
