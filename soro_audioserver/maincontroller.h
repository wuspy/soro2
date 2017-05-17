#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <QObject>
#include <QCoreApplication>
#include <QTimerEvent>

#include "audioserver.h"

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

    QString _id;
    AudioServer *_audioServer = nullptr;
    SettingsModel *_settingsModel = nullptr;
};

} // namespace Soro

#endif // MAINCONTROLLER_H
