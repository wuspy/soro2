#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QApplication>
#include <QUdpSocket>
#include <QTimerEvent>

#include "soro_core/rosnodelist.h"

#include "settingsmodel.h"
#include "mainwindowcontroller.h"
#include "masterconnectionstatuscontroller.h"
#include "mastervideocontroller.h"

namespace Soro {

/* Class to forward audio/video streams to all mission control computers.
 */
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
    static MainController *_self;

    int _rosSpinTimerId;

    SettingsModel *_settings = nullptr;
    CameraSettingsModel *_cameraSettings = nullptr;
    QQmlEngine *_qmlEngine = nullptr;
    MasterVideoController *_masterVideoController = nullptr;
    MainWindowController *_mainWindowController = nullptr;
    MasterConnectionStatusController *_masterConnectionStatusController = nullptr;
    RosNodeList *_rosNodeList = nullptr;
};

} // namespace Soro

#endif // MAINCONTROLLER_H
