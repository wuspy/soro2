#ifndef ROSCORECONTROLLER_H
#define ROSCORECONTROLLER_H

#include <QObject>
#include <QProcess>

namespace Soro {

/* Class to start and monitor roscore.
 *
 * This class will execute roscore when it is created, using the master port defined
 * by SORO_NET_ROS_MASTER_PORT, and is responsible for terminating it upon destruction
 * so roscore does not become orphaned.
 *
 * This class will call panic() if roscore terminates unexpectedly. It does not emit signals
 * or otherwise provide an API to access such information about the status of the process.
 */
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
