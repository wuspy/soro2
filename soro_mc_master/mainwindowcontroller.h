#ifndef MAINWINDOWCONTROLLER_H
#define MAINWINDOWCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QQuickWindow>

namespace Soro {

class MainWindowController : public QObject
{
    Q_OBJECT
public:
    explicit MainWindowController(QQmlEngine *engine, QObject *parent = 0);

public Q_SLOTS:
    void onBitrateUpdated(quint64 bitrateUp, quint64 bitrateDown);
    void onLatencyUpdated(quint32 latency);
    void onConnectedChanged(bool connected);

private:
    QQuickWindow *_window = nullptr;
};

} // namespace Soro

#endif // MAINWINDOWCONTROLLER_H
