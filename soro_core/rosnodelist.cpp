#include "rosnodelist.h"
#include "logger.h"

#include <QProcess>
#include <QHostInfo>

#define LogTag "RosNodeList"

namespace Soro {

RosNodeList::RosNodeList(int pollInterval, QObject *parent) : QObject(parent)
{
    _nodePollTimerId = startTimer(pollInterval);
}

void RosNodeList::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _nodePollTimerId)
    {
        QProcess *process = new QProcess(this);
        process->start("/bin/bash", QStringList() << "-c" << "source /opt/ros/kinetic/setup.bash; exec rosnode list -a");

        connect(process, static_cast<void (QProcess::*)(int)>(&QProcess::finished), this, [process, this](int exitCode)
        {
            if (exitCode != 0)
            {
                Logger::logError(LogTag, "rosnode list -a exited with code " + QString::number(exitCode));
            }
            else {
                _nodes.clear();
                while (process->canReadLine())
                {
                    QString line = QString(process->readLine());
                    if (line.startsWith("http://"))
                    {
                        Node node;
                        node.address = QHostInfo::fromName(line.mid(7, line.indexOf(":", 7) - 7)).addresses()[0];
                        node.name = line.mid(line.indexOf("\t") + 1).trimmed();
                        _nodes.append(node);
                    }
                }
                if (!_nodes.isEmpty())
                {
                    Q_EMIT nodesUpdated();
                }
            }
            disconnect(process, 0, this, 0);
            delete process;
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
