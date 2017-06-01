#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <QObject>
#include <QCoreApplication>
#include <QTimerEvent>

#include "settingsmodel.h"
#include "drivecontroller.h"

namespace Soro {

class MainController : public QObject
{
    Q_OBJECT
public:
    static void init(QCoreApplication *app);
    static void panic(QString tag, QString message);

    static QString getId();

private:
    explicit MainController(QObject *parent=0);
    static MainController *_self;

    SettingsModel *_settingsModel = nullptr;
    DriveController *_driveController = nullptr;
};

} // namespace Soro

#endif // MAINCONTROLLER_H
