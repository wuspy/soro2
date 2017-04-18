#ifndef ROSCORECONTROLLER_H
#define ROSCORECONTROLLER_H

#include <QObject>
#include <QProcess>

namespace Soro {

class RosCoreController : public QObject
{
    Q_OBJECT
public:
    explicit RosCoreController(QObject *parent = 0);
    ~RosCoreController();

private Q_SLOTS:
    void roscoreProcessStateChanged(QProcess::ProcessState state);

private:
    QProcess _roscoreProcess;
};

} // namespace Soro

#endif // ROSCORECONTROLLER_H
