#ifndef ROSCORECONTROLLER_H
#define ROSCORECONTROLLER_H

#include <QObject>
#include <QProcess>

class RosCoreController : public QObject
{
    Q_OBJECT
public:
    explicit RosCoreController(QObject *parent = 0);

Q_SIGNALS:

public Q_SLOTS:

private:
    QProcess _roscoreProcess;
};

#endif // ROSCORECONTROLLER_H
