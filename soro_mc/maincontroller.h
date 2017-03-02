#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <QObject>
#include <QApplication>
#include <QQmlEngine>
#include <QQuickWindow>

#include <Qt5GStreamer/QGst/Element>
#include "libsoromc/settingsmodel.h"
#include "libsoromc/camerasettingsmodel.h"
#include "gamepadcontroller.h"
#include "mastercontroller.h"
#include "rosconnectioncontroller.h"
#include "gamepadcontroller.h"
#include "mainwindowcontroller.h"
#include "qquickgstreamersurface.h"
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
    static QString getMissionControlId();
    static MainWindowController* getMainWindowController();

    static void logDebug(QString tag, QString message);
    static void logInfo(QString tag, QString message);
    static void logWarning(QString tag, QString message);
    static void logError(QString tag, QString message);
    static void logFatal(QString tag, QString message);

private:
    enum LogLevel {
        LogLevelDebug = 0,
        LogLevelInfo,
        LogLevelWarning,
        LogLevelError,
        LogLevelFatal
    };

    QGst::ElementPtr _sink1;
    QGst::ElementPtr _sink2;

    int _sinkConfig = 1;

    explicit MainController(QObject *parent=0);
    QString genId();
    void log(LogLevel level, QString tag, QString message);

    static MainController *_self;

    QQuickGStreamerSurface *_vidSurface1;
    QQuickGStreamerSurface *_vidSurface2;
    QQuickGStreamerSurface *_vidSurface3;
    QString _mcId;
    QQmlEngine *_qmlEngine;
    QQuickWindow *_window;

    GamepadController* _gamepadController = nullptr;
    MasterController *_masterController = nullptr;
    RosConnectionController *_rosConnectionController = nullptr;
    SettingsModel* _settingsModel = nullptr;
    CameraSettingsModel *_cameraSettingsModel = nullptr;
    MainWindowController *_mainWindowController = nullptr;
    DriveControlSystem *_driveSystem = nullptr;

private slots:
    void initInternal();
    void gamepadButtonPressed(SDL_GameControllerButton button, bool pressed);
};

} // namespace Soro

#endif // MAINCONTROLLER_H
