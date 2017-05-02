#ifndef ROSNODELIST_H
#define ROSNODELIST_H

#include <QObject>
#include <QHostAddress>
#include <QTimerEvent>

namespace Soro {

/* Class which uses the rosnode command to introspect the current ROS network, and obtain
 * the names and URI's of all connected nodes
 */
class RosNodeList : public QObject
{
    Q_OBJECT
public:
    explicit RosNodeList(int pollInterval, QObject *parent = 0);

    struct Node
    {
        QString name;
        QHostAddress address;
    };

    QVector<Node> getNodes() const;
    QHostAddress getAddressForNode(QString nodeName) const;

Q_SIGNALS:
    void nodesUpdated();

protected:
    void timerEvent(QTimerEvent *e);

private:
    QVector<Node> _nodes;
    int _nodePollTimerId;
};

} // namespace Soro

#endif // ROSNODELIST_H
