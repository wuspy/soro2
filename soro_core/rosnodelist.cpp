#include "rosnodelist.h"
#include "logger.h"

#include <QProcess>
#include <QHostInfo>

#define LogTag "RosNodeList"

namespace Soro {

RosNodeList::RosNodeList(int pollInterval, QObject *parent) : QObject(parent)
{
    _nodePollTimerId = startTimer(pollInterval);
    _runningProcess = nullptr;
}

RosNodeList::~RosNodeList()
{
    // Ensure the rosnode process is terminated
    if (_runningProcess)
    {
        _runningProcess->terminate();
    }
}

void RosNodeList::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _nodePollTimerId)
    {
        if (_runningProcess) {
            Logger::logWarn(LogTag, "'rosnode list -a' has been running for longer than a full poll interval");
            return;
        }

        _runningProcess = new QProcess(this);
        _runningProcess->start("/bin/bash", QStringList() << "-c" << "source /opt/ros/kinetic/setup.bash; exec rosnode list -a");

        // This lambda function will run when the process has finished, where we will then parse its output to determine
        // the names and addresses of all nodes on the ROS network
        connect(_runningProcess, static_cast<void (QProcess::*)(int)>(&QProcess::finished), this, [this](int exitCode)
        {
            if (exitCode != 0)
            {
                Logger::logError(LogTag, "rosnode list -a exited with code " + QString::number(exitCode));
            }
            else
            {
                _nodes.clear();
                while (_runningProcess->canReadLine())
                {
                    QString line = QString(_runningProcess->readLine());
                    if (line.startsWith("http://"))
                    {
                        Node node;
                        QString address = line.mid(7, line.indexOf(":", 7) - 7);
                        node.address = QHostAddress(address);
                        if (node.address == QHostAddress::Null)
                        {
                            // Try looking up this address as a hostname
                            QList<QHostAddress> addresses = QHostInfo::fromName(line.mid(7, line.indexOf(":", 7) - 7)).addresses();
                            if (addresses.size() == 0)
                            {
                                Logger::logError(LogTag, "Cannot resolve IP for " + address);
                                continue;
                            }
                            node.address = addresses[0];
                        }

                        node.name = line.mid(line.indexOf("\t") + 1).trimmed();
                        _nodes.append(node);
                    }
                }
                if (!_nodes.isEmpty())
                {
                    Q_EMIT nodesUpdated();
                }
            }

            // Delete the process
            disconnect(_runningProcess, 0, this, 0);
            delete _runningProcess;
            _runningProcess = nullptr;
        });
    }
}

QVector<RosNodeList::Node> RosNodeList::getNodes() const
{
    return _nodes;
}

QHostAddress RosNodeList::getAddressForNode(QString nodeName) const
{
    for (RosNodeList::Node node : _nodes)
    {
        if (node.name == nodeName) return node.address;
    }
    return QHostAddress::Null;
}

} // namespace Soro
