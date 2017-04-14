#ifndef MAINWINDOWCONTROLLER_H
#define MAINWINDOWCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QQuickWindow>

class MainWindowController : public QObject
{
    Q_OBJECT
public:
    explicit MainWindowController(QQmlEngine *engine, QObject *parent = 0);

private Q_SLOTS:
    void onBitrateUpdated(quint64 bitrateUp, quint64 bitrateDown);
    void onLatencyUpdated(quint32 latency);
    void onConnectedChanged(bool connected);

private:
    QQuickWindow *_window = nullptr;
};

#endif // MAINWINDOWCONTROLLER_H
