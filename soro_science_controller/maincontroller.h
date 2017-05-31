#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <QObject>
#include <QCoreApplication>
#include <QUdpSocket>
#include <QTimerEvent>

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
};

} // namespace Soro

#endif // MAINCONTROLLER_H
