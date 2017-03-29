#ifndef MASTERCONTROLLER_H
#define MASTERCONTROLLER_H

#include <QObject>
#include <QProcess>

namespace Soro {

class MasterController: public QObject
{
    Q_OBJECT
public:
    MasterController(QObject *parent=0);
    ~MasterController();

private slots:
    void processStarted();
    void processFinished(int exitCode);

private:
    QProcess _masterProcess;
};

} // namespace Soro

#endif // MASTERCONTROLLER_H
