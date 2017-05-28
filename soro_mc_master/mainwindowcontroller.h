#ifndef MAINWINDOWCONTROLLER_H
#define MAINWINDOWCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QQuickWindow>
#include <QHostAddress>

namespace Soro {

class MainWindowController : public QObject
{
    Q_OBJECT
public:
    explicit MainWindowController(QQmlEngine *engine, QObject *parent = 0);

public Q_SLOTS:
    void onDataRateUpdated(quint64 rateFromRover);
    void onLatencyUpdated(quint32 latency);
    void onConnected();
    void onDisconnected();
    void onAudioBounceAddressesChanged(const QHash<QString, QHostAddress>& addresses);
    void onVideoBounceAddressesChanged(const QHash<QString, QHostAddress>& addresses);

private:
    QQuickWindow *_window = nullptr;
};

} // namespace Soro

#endif // MAINWINDOWCONTROLLER_H
