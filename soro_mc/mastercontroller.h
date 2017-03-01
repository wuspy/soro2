#ifndef MASTERCONTROLLER_H
#define MASTERCONTROLLER_H

#include <QObject>
#include <QProcess>

namespace Soro {

class MasterController: public QObject
{
    Q_OBJECT
public:
    MasterController();

private:
    QProcess _masterProcess;

};

} // namespace Soro

#endif // MASTERCONTROLLER_H
