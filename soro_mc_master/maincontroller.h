#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QApplication>
#include <QUdpSocket>
#include <QTimerEvent>

#include "settingsmodel.h"
#include "mainwindowcontroller.h"
#include "masterconnectionstatuscontroller.h"
#include "mastervideocontroller.h"
#include "masteraudiocontroller.h"

namespace Soro {

class MainController : public QObject
{
    Q_OBJECT
public:
    static void init(QApplication *app);
    static void panic(QString tag, QString message);

    static QString getId();

private:
    explicit MainController(QObject *parent=0);
    static MainController *_self;

    SettingsModel *_settings = nullptr;
    CameraSettingsModel *_cameraSettings = nullptr;
    QQmlEngine *_qmlEngine = nullptr;
    MasterVideoController *_masterVideoController = nullptr;
    MasterAudioController *_masterAudioController = nullptr;
    MainWindowController *_mainWindowController = nullptr;
    MasterConnectionStatusController *_masterConnectionStatusController = nullptr;
};

} // namespace Soro

#endif // MAINCONTROLLER_H
