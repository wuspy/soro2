#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <QObject>
#include <QApplication>
#include <QQmlEngine>
#include <QUdpSocket>

#include "libsoromc/settingsmodel.h"
#include "libsoromc/camerasettingsmodel.h"

#include "gamepadcontroller.h"
#include "mastercontroller.h"
#include "gamepadcontroller.h"
#include "mainwindowcontroller.h"
#include "drivecontrolsystem.h"

namespace Soro {

class MainController : public QObject
{
    Q_OBJECT
public:
    static void init(QApplication *app);
    static void panic(QString message);

    static GamepadController* getGamepadController();
    static SettingsModel* getSettingsModel();
    static CameraSettingsModel* getCameraSettingsModel();
    static QString getMissionControlId();
    static MainWindowController* getMainWindowController();
    static ros::NodeHandle* getNodeHandle();

signals:
    void initialized();

private:

    explicit MainController(QObject *parent=0);
    QString genId();

    static MainController *_self;

    QUdpSocket *_rosInitUdpSocket = nullptr;
    ros::NodeHandle *_nodeHandle = nullptr;
    QString _mcId;
    QQmlEngine *_qmlEngine = nullptr;
    GamepadController* _gamepadController = nullptr;
    MasterController *_masterController = nullptr;
    SettingsModel* _settingsModel = nullptr;
    CameraSettingsModel *_cameraSettingsModel = nullptr;
    MainWindowController *_mainWindowController = nullptr;
    DriveControlSystem *_driveControlSystem = nullptr;

private slots:
    void initInternal();
    void onRosMasterFound();
    void rosInitUdpReadyRead();
};

} // namespace Soro

#endif // MAINCONTROLLER_H
