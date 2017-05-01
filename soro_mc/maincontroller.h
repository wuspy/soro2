#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <QObject>
#include <QApplication>
#include <QQmlEngine>
#include <QUdpSocket>
#include <QTimerEvent>

#include "settingsmodel.h"
#include "soro_core/camerasettingsmodel.h"
#include "soro_core/mediaprofilesettingsmodel.h"
#include "soro_core/broadcaster.h"

#include "gamepadcontroller.h"
#include "gamepadcontroller.h"
#include "mainwindowcontroller.h"
#include "connectionstatuscontroller.h"
#include "drivecontrolsystem.h"
#include "audiocontroller.h"
#include "videocontroller.h"
#include "armcontrolsystem.h"
#include "bindssettingsmodel.h"

namespace Soro {

class MainController : public QObject
{
    Q_OBJECT
public:
    static void init(QApplication *app);
    static void panic(QString tag, QString message);

    static QString getId();

protected:
    void timerEvent(QTimerEvent *e);

private:

    explicit MainController(QObject *parent=0);
    QString genId();

    static MainController *_self;

    int _rosSpinTimerId;

    QUdpSocket *_rosInitUdpSocket = nullptr;
    QString _mcId;
    QQmlEngine *_qmlEngine = nullptr;
    GamepadController* _gamepadController = nullptr;
    SettingsModel* _settingsModel = nullptr;
    AudioController *_audioController = nullptr;
    VideoController *_videoController = nullptr;
    CameraSettingsModel *_cameraSettingsModel = nullptr;
    BindsSettingsModel *_bindsSettingsModel = nullptr;
    MediaProfileSettingsModel *_mediaProfileSettingsModel = nullptr;
    MainWindowController *_mainWindowController = nullptr;
    DriveControlSystem *_driveControlSystem = nullptr;
    ConnectionStatusController *_connectionStatusController = nullptr;
    ArmControlSystem * _armControlSystem = nullptr;
    Broadcaster *_mcBroadcaster = nullptr;
};

} // namespace Soro

#endif // MAINCONTROLLER_H
